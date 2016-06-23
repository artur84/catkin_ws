/*********************************************************************
 * Author: Arturo Escobedo
 *********************************************************************/
#include <user_dir_recovery/user_dir_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(user_dir_recovery, UserDirRecovery,
		user_dir_recovery::UserDirRecovery, nav_core::RecoveryBehavior)

namespace gm = geometry_msgs;
namespace cmap = costmap_2d;
namespace blp = base_local_planner;
using std::vector;
using std::max;

namespace user_dir_recovery
{

UserDirRecovery::UserDirRecovery() :
		global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(
				false)
{
}

UserDirRecovery::~UserDirRecovery()
{
	delete world_model_;
}

void UserDirRecovery::initialize(std::string name, tf::TransformListener* tf,
		cmap::Costmap2DROS* global_cmap, cmap::Costmap2DROS* local_cmap)
{
	ROS_ASSERT(!initialized_);
	name_ = name;
	tf_ = tf;
	local_costmap_ = local_cmap;
	global_costmap_ = global_cmap;
	local_costmap_->getCostmapCopy(costmap_);
	world_model_ = new blp::CostmapModel(costmap_);
	destination_inference_enable_client_ = nh_.serviceClient<std_srvs::Empty>(
			"destination_inference/enable");
	destination_inference_disable_client_ = nh_.serviceClient<std_srvs::Empty>(
			"destination_inference/disable");
	pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
	wheelchair_talk_pub_ = nh_.advertise<std_msgs::String>("recognizer/output",
			1);
	input_sub_ = nh_.subscribe("user_vel", 1,
			&UserDirRecovery::_input_vel_callback_, this);
	input_voice_sub_ = nh_.subscribe("recognizer/output", 1,
			&UserDirRecovery::_input_voice_callback_, this);

	ros::NodeHandle private_nh("~/" + name);
	received_move_flag_ = false;

	{
		bool found = true;
		if (!found)
		{
			ROS_FATAL_STREAM(
					"Didn't find twist parameters in " << private_nh.getNamespace());
			ros::shutdown();
		}
	}

	private_nh.param("duration", duration_, 1.0);
	private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
	private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
	private_nh.param("linear_acceleration_limit", linear_acceleration_limit_,
			4.0);
	private_nh.param("angular_acceleration_limit", angular_acceleration_limit_,
			3.2);
	private_nh.param("controller_frequency", controller_frequency_, 20.0);
	private_nh.param("simulation_inc", simulation_inc_,
			1 / controller_frequency_);

	ROS_INFO_STREAM_NAMED("recovery",
			"Initialized user dir recovery with: " << duration_);

	initialized_ = true;
}

gm::Twist scaleTwist(const gm::Twist& twist, const double scale)
{
	gm::Twist t;
	t.linear.x = twist.linear.x * scale;
	t.linear.y = twist.linear.y * scale;
	t.angular.z = twist.angular.z * scale;
	return t;
}

gm::Pose2D forwardSimulate(const gm::Pose2D& p, const gm::Twist& twist,
		const double t = 1.0)
{
	gm::Pose2D p2;
	p2.x = p.x + twist.linear.x * t;
	p2.y = p.y + twist.linear.y * t;
	p2.theta = p.theta + twist.angular.z * t;
	return p2;
}

/// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
double UserDirRecovery::normalizedPoseCost(const gm::Pose2D& pose) const
{
	gm::Point p;
	p.x = pose.x;
	p.y = pose.y;
	vector<gm::Point> oriented_footprint;
	local_costmap_->getOrientedFootprint(pose.x, pose.y, pose.theta,
			oriented_footprint);
	const double c = world_model_->footprintCost(p, oriented_footprint,
			local_costmap_->getInscribedRadius(),
			local_costmap_->getCircumscribedRadius());
	return c < 0 ? 1e9 : c;
}

/// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
/// d seconds if we follow twist
/// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
/// the first k of those d seconds, but this is not done
double UserDirRecovery::nonincreasingCostInterval(const gm::Pose2D& current,
		const gm::Twist& twist) const
{
	double cost = normalizedPoseCost(current);
	double t; // Will hold the first time that is invalid
	for (t = simulation_inc_; t <= duration_; t += simulation_inc_)
	{
		const double next_cost = normalizedPoseCost(
				forwardSimulate(current, twist, t));
		if (next_cost > cost)
		{
			ROS_DEBUG_STREAM_NAMED("recovery",
					"Cost at " << t << " and pose " << forwardSimulate(current, twist, t) << " is " << next_cost << " which is greater than previous cost " << cost);
			break;
		}
		cost = next_cost;
	}

	return t - simulation_inc_;
}

double linearSpeed(const gm::Twist& twist)
{
	return sqrt(
			twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
}

double angularSpeed(const gm::Twist& twist)
{
	return fabs(twist.angular.z);
}

// Scale twist so we can stop in the given time, and so it's within the max velocity
gm::Twist UserDirRecovery::scaleGivenAccelerationLimits(const gm::Twist& twist,
		const double time_remaining) const
{
	const double linear_speed = linearSpeed(twist);
	const double angular_speed = angularSpeed(twist);
	const double linear_acc_scaling = linear_speed
			/ (time_remaining * linear_acceleration_limit_);
	const double angular_acc_scaling = angular_speed
			/ (time_remaining * angular_acceleration_limit_);
	const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
	const double linear_vel_scaling = linear_speed / linear_speed_limit_;
	const double angular_vel_scaling = angular_speed / angular_speed_limit_;
	const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
	return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
}

// Get pose in local costmap frame
gm::Pose2D UserDirRecovery::getCurrentLocalPose() const
{
	tf::Stamped<tf::Pose> p;
	local_costmap_->getRobotPose(p);
	gm::Pose2D pose;
	pose.x = p.getOrigin().x();
	pose.y = p.getOrigin().y();
	pose.theta = tf::getYaw(p.getRotation());
	return pose;
}

void UserDirRecovery::runBehavior()
{
	ROS_ASSERT(initialized_);
	std_srvs::Empty srv;
	destination_inference_disable_client_.call(srv); //This will avoid the destination inference to send a
													 //new goal to the move_base while this recovery behavior is running
	received_move_flag_ = false; // To clean this flag an wait until it is set in the voice callback as true

	// Figure out how long we can safely run the behavior
	const gm::Pose2D& current = getCurrentLocalPose();
	local_costmap_->getCostmapCopy(costmap_); // This affects world_model_, which is used in the next step
	//takes an snapshot of the input commanded by the user.

	ros::Rate r(controller_frequency_);
	geometry_msgs::Twist vel;
	// We'll now apply this twist open-loop for d seconds (scaled so we can guarantee stopping at the end)
	ask_for_help();
	ROS_INFO_NAMED("recovery",
			"In recovery_behavior: I will wait until the user says go!!");
	while (true)
	{

		// Computes the maximum interval that input_vel_ can be applied without collission.
		if (received_move_flag_)
		{
			for (double t = 0; t < duration_; t += 1 / controller_frequency_)
			{
				vel.angular.z = input_vel_.angular.z / 3;
				vel.linear.x = input_vel_.linear.x / 3;
//				base_frame_twist_ = scaleGivenAccelerationLimits(vel,
//						duration_ - t);
				ROS_INFO_NAMED("recovery", "Applying (%.2f, %.2f)",
						vel.linear.x,
						vel.angular.z);
				pub_.publish(vel);
				r.sleep();
			}
			vel.angular.z = 0, vel.linear.x = 0;//Stop before returning to move_base main loop
			pub_.publish(vel);
			ROS_DEBUG_NAMED("recovery", "Applying (%.2f, %.2f)",
					vel.linear.x, vel.angular.z);
			destination_inference_enable_client_.call(srv); //Before returning we will activate again the destination inference module
			r.sleep();
			return;
		}
		else
		{
			ROS_DEBUG_NAMED("recovery",
					"Waiting for the user to say go to start recovery behavior");
			r.sleep();
		}

	}
}

void UserDirRecovery::_input_vel_callback_(
		const geometry_msgs::TwistConstPtr& vel)
{
	/***
	 * Receives the input velocity command (user's desired velocity).
	 */
	input_vel_ = *vel;
	ROS_DEBUG_NAMED("recovery",
			"recovery callback twist=(%.2f,%.2f)",
			input_vel_.linear.x, input_vel_.angular.z);

}

void UserDirRecovery::_input_voice_callback_(
		const std_msgs::StringConstPtr& voice)
{
	/***
	 * Receives the input velocity command (user's desired velocity).
	 */
	input_voice_ = *voice;
	ROS_INFO_NAMED("recovery", "In ROS input_vel_callback: %s", &input_voice_.data);
	if (voice->data == "go" || voice->data == "back" || voice->data == "join")
		received_move_flag_ = true;

}

void UserDirRecovery::ask_for_help(void)
{
	/***
	 * Sends a message to the wheelchair talk to ask for help to the user
	 */
	std_msgs::String message;
	message.data = "help";
	wheelchair_talk_pub_.publish(message);

}
} // namespace user_dir_recovery
