#include <ui_local_planner/trajectory_planner_ros.h>

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <ui_local_planner/goal_functions.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(base_local_planner, TrajectoryPlannerROS,
		ui_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace ui_local_planner
{

void TrajectoryPlannerROS::reconfigureCB(UILocalPlannerConfig &config,
		uint32_t level)
{
	/**
	 * @brief Callback to update the local planner's parameters based on dynamic reconfigure
	 */
	if (setup_ && config.restore_defaults)
	{
		config = default_config_;
		//Avoid looping
		config.restore_defaults = false;
	}
	if (!setup_)
	{
		default_config_ = config;
		setup_ = true;
	}
	tc_->reconfigure(config);
}

TrajectoryPlannerROS::TrajectoryPlannerROS() :
		world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(
				false), initialized_(false), odom_helper_("odom")
{
	/**
	 * @brief  Default constructor for the ros wrapper
	 */
}

TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name,
		tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
		world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(
				false), initialized_(false), odom_helper_("odom")
{
	/**
	 * @brief  Constructs the ros wrapper
	 * @param name The name to give this instance of the trajectory planner
	 * @param tf A pointer to a transform listener
	 * @param costmap The cost map to use for assigning costs to trajectories
	 */
	//initialize the planner
	initialize(name, tf, costmap_ros);
}

void TrajectoryPlannerROS::initialize(std::string name,
		tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
	/**
	 * @brief  Constructs the ros wrapper
	 * @param name The name to give this instance of the trajectory planner
	 * @param tf A pointer to a transform listener
	 * @param costmap The cost map to use for assigning costs to trajectories
	 */
	if (!isInitialized())
	{

		ros::NodeHandle private_nh("~/" + name);
		g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1); //The complete global plan from A* or dijkstra
		l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		partial_plan_pub_ = private_nh.advertise<nav_msgs::Path>("partial_plan",
				1);
		//goal_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("goal_marker", 1); //to display a goal pose in the map ( I should increase it to more than one after success).

		tf_ = tf;
		costmap_ros_ = costmap_ros;
		rot_stopped_velocity_ = 1e-2;
		trans_stopped_velocity_ = 1e-2;
		double sim_time, sim_granularity, angular_sim_granularity;
		int vx_samples, vtheta_samples;
		double pdist_scale, gdist_scale, occdist_scale, heading_lookahead,
				oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
		bool holonomic_robot, dwa, simple_attractor, heading_scoring;
		double heading_scoring_timestep;
		double max_vel_x, min_vel_x;
		double backup_vel;
		double stop_time_buffer;
		std::string world_model_type;
		rotating_to_goal_ = false;

		//initialize the copy of the costmap the controller will use
		costmap_ros_->getCostmapCopy(costmap_);

		global_frame_ = costmap_ros_->getGlobalFrameID();
		robot_base_frame_ = costmap_ros_->getBaseFrameID();
		private_nh.param("prune_plan", prune_plan_, true);

		private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
		private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);

		//we'll get the parameters for the robot radius from the costmap we're associated with
		inflation_radius_ = costmap_ros_->getInflationRadius();

		private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
		private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
		private_nh.param("acc_lim_th", acc_lim_theta_, 3.2);

		private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

		private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_,
				false);

		//Since I screwed up nicely in my documentation, I'm going to add errors
		//informing the user if they've set one of the wrong parameters
		if (private_nh.hasParam("acc_limit_x"))
			ROS_ERROR(
					"You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

		if (private_nh.hasParam("acc_limit_y"))
			ROS_ERROR(
					"You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

		if (private_nh.hasParam("acc_limit_th"))
			ROS_ERROR(
					"You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

		//Assuming this planner is being run within the navigation stack, we can
		//just do an upward search for the frequency at which its being run. This
		//also allows the frequency to be overwritten locally.
		std::string controller_frequency_param_name;
		if (!private_nh.searchParam("controller_frequency",
				controller_frequency_param_name))
			sim_period_ = 0.05;
		else
		{
			double controller_frequency = 0;
			private_nh.param(controller_frequency_param_name,
					controller_frequency, 20.0);
			if (controller_frequency > 0)
				sim_period_ = 1.0 / controller_frequency;
			else
			{
				ROS_WARN(
						"A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
				sim_period_ = 0.05;
			}
		}
		ROS_INFO("Sim period is set to %.2f", sim_period_);

		private_nh.param("sim_time", sim_time, 1.0);
		private_nh.param("sim_granularity", sim_granularity, 0.025);
		private_nh.param("angular_sim_granularity", angular_sim_granularity,
				sim_granularity);
		private_nh.param("vx_samples", vx_samples, 3);
		private_nh.param("vtheta_samples", vtheta_samples, 20);

		private_nh.param("path_distance_bias", pdist_scale, 0.6);
		private_nh.param("goal_distance_bias", gdist_scale, 0.8);
		private_nh.param("occdist_scale", occdist_scale, 0.01);

		bool meter_scoring;
		if (!private_nh.hasParam("meter_scoring"))
		{
			ROS_WARN(
					"ui_local planner initialized with param meter_scoring not set. Set it to true to make your settins robust against changes of costmap resolution.");
		}
		else
		{
			private_nh.param("meter_scoring", meter_scoring, false);

			if (meter_scoring)
			{
				//if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
				double resolution = costmap_ros_->getResolution();
				gdist_scale *= resolution;
				pdist_scale *= resolution;
				occdist_scale *= resolution;
			}
			else
			{
				ROS_WARN(
						"ui_local planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
			}
		}

		private_nh.param("heading_lookahead", heading_lookahead, 0.325);
		private_nh.param("oscillation_reset_dist", oscillation_reset_dist,
				0.05);
		private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
		private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
		private_nh.param("holonomic_robot", holonomic_robot, true);
		private_nh.param("max_vel_x", max_vel_x, 0.5);
		private_nh.param("min_vel_x", min_vel_x, 0.1);

		double max_rotational_vel;
		private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
		max_vel_th_ = max_rotational_vel;
		min_vel_th_ = -1.0 * max_rotational_vel;
		private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_,
				0.4);

		backup_vel = -0.1;
		if (private_nh.getParam("backup_vel", backup_vel))
			ROS_WARN(
					"The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

		//if both backup_vel and escape_vel are set... we'll use escape_vel
		private_nh.getParam("escape_vel", backup_vel);

		if (backup_vel >= 0.0)
			ROS_WARN(
					"You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

		private_nh.param("world_model", world_model_type,
				std::string("costmap"));
		private_nh.param("dwa", dwa, true);
		private_nh.param("heading_scoring", heading_scoring, false);
		private_nh.param("heading_scoring_timestep", heading_scoring_timestep,
				0.8);

		simple_attractor = false;

		//parameters for using the freespace controller
		double min_pt_separation, max_obstacle_height, grid_resolution;
		private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
		private_nh.param("point_grid/min_pt_separation", min_pt_separation,
				0.01);
		private_nh.param("point_grid/max_obstacle_height", max_obstacle_height,
				2.0);
		private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

		// Parameters for the user intentions vel handler
		bool ui_scoring; //If it is one, the trajectory scoring will take into account the input from the user
		double user_vel_cost_scale; //weighting factor
		double face_dir_cost_scale; //weighting factor for the face direction
		private_nh.param("ui_scoring", ui_scoring, false);
		private_nh.param("user_vel_cost_scale", user_vel_cost_scale, 0.00); //
		private_nh.param("face_dir_cost_scale", face_dir_cost_scale, 0.00);

		ROS_ASSERT_MSG(world_model_type == "costmap",
				"At this time, only costmap world models are supported by this controller");
		world_model_ = new CostmapModel(costmap_);
		std::vector<double> y_vels = loadYVels(private_nh);
		//XXX create a new trajectory controller tc_ object
		tc_ = new TrajectoryPlanner(*world_model_, costmap_,
				costmap_ros_->getRobotFootprint(), acc_lim_x_, acc_lim_y_,
				acc_lim_theta_, sim_time, sim_granularity, vx_samples,
				vtheta_samples, pdist_scale, gdist_scale, occdist_scale,
				heading_lookahead, oscillation_reset_dist, escape_reset_dist,
				escape_reset_theta, holonomic_robot, max_vel_x, min_vel_x,
				max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel, dwa,
				heading_scoring, heading_scoring_timestep, meter_scoring,
				simple_attractor, y_vels, stop_time_buffer, sim_period_,
				angular_sim_granularity, ui_scoring, user_vel_cost_scale,
				face_dir_cost_scale);

		map_viz_.initialize(name,
				boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3,
						_4, _5, _6));
		initialized_ = true;

		dsrv_ = new dynamic_reconfigure::Server<UILocalPlannerConfig>(
				private_nh);
		dynamic_reconfigure::Server<UILocalPlannerConfig>::CallbackType cb =
				boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
		user_vel_handler_.initReader("user_vel");
//		face_dir_handler_.initReader("face_dir");
		private_nh.param("publish_traj_markers", publish_traj_markers_, false);
		if (publish_traj_markers_)
			markers_handler_.initWriter(global_frame_);

	}
	else
	{
		ROS_WARN("This planner has already been initialized, doing nothing");
	}
}

std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node)
{
	std::vector<double> y_vels;

	std::string y_vel_list;
	if (node.getParam("y_vels", y_vel_list))
	{
		typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		boost::char_separator<char> sep("[], ");
		tokenizer tokens(y_vel_list, sep);

		for (tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++)
		{
			y_vels.push_back(atof((*i).c_str()));
		}
	}
	else
	{
		//if no values are passed in, we'll provide defaults
		y_vels.push_back(-0.3);
		y_vels.push_back(-0.1);
		y_vels.push_back(0.1);
		y_vels.push_back(0.3);
	}

	return y_vels;
}

TrajectoryPlannerROS::~TrajectoryPlannerROS()
{
	/**
	 * @brief  Destructor for the wrapper
	 */
	//make sure to clean things up
	delete dsrv_;

	if (tc_ != NULL)
		delete tc_;

	if (world_model_ != NULL)
		delete world_model_;
}

bool TrajectoryPlannerROS::stopWithAccLimits(
		const tf::Stamped<tf::Pose>& global_pose,
		const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel)
{
	/**
	 * @brief Stop the robot taking into account acceleration limits
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	//slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
	//but we'll use a tenth of a second to be consistent with the implementation of the local planner.
	double vx =
			sign(robot_vel.getOrigin().x())
					* std::max(0.0,
							(fabs(robot_vel.getOrigin().x())
									- acc_lim_x_ * sim_period_));
	double vy =
			sign(robot_vel.getOrigin().y())
					* std::max(0.0,
							(fabs(robot_vel.getOrigin().y())
									- acc_lim_y_ * sim_period_));

	double vel_yaw = tf::getYaw(robot_vel.getRotation());
	double vth = sign(vel_yaw)
			* std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

	//we do want to check whether or not the command is valid
	double yaw = tf::getYaw(global_pose.getRotation());
	bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(),
			global_pose.getOrigin().getY(), yaw, robot_vel.getOrigin().getX(),
			robot_vel.getOrigin().getY(), vel_yaw, vx, vy, vth);

	//if we have a valid command, we'll pass it on, otherwise we'll command all zeros
	if (valid_cmd)
	{
		ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy,
				vth);
		cmd_vel.linear.x = vx;
		cmd_vel.linear.y = vy;
		cmd_vel.angular.z = vth;
		return true;
	}

	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.angular.z = 0.0;
	return false;
}

bool TrajectoryPlannerROS::rotateToGoal(
		const tf::Stamped<tf::Pose>& global_pose,
		const tf::Stamped<tf::Pose>& robot_vel, double goal_th,
		geometry_msgs::Twist& cmd_vel)
{
	/**
	 * @brief Once a goal position is reached... rotate to the goal orientation
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  goal_th The desired th value for the goal
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	double yaw = tf::getYaw(global_pose.getRotation());
	double vel_yaw = tf::getYaw(robot_vel.getRotation());
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

	double v_theta_samp =
			ang_diff > 0.0 ?
					std::min(max_vel_th_,
							std::max(min_in_place_vel_th_, ang_diff)) :
					std::max(min_vel_th_,
							std::min(-1.0 * min_in_place_vel_th_, ang_diff));

	//take the acceleration limits of the robot into account
	double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
	double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

	v_theta_samp = sign(v_theta_samp)
			* std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

	//we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
	double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff));

	v_theta_samp = sign(v_theta_samp)
			* std::min(max_speed_to_stop, fabs(v_theta_samp));

	// Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
	v_theta_samp =
			v_theta_samp > 0.0 ?
					std::min(max_vel_th_,
							std::max(min_in_place_vel_th_, v_theta_samp)) :
					std::max(min_vel_th_,
							std::min(-1.0 * min_in_place_vel_th_,
									v_theta_samp));

	//we still want to lay down the footprint of the robot and check if the action is legal
	//XXX: here is how you can use it to check if turning is safe.
	bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(),
			global_pose.getOrigin().getY(), yaw, robot_vel.getOrigin().getX(),
			robot_vel.getOrigin().getY(), vel_yaw, 0.0, 0.0, v_theta_samp);

	ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d",
			v_theta_samp, valid_cmd);

	if (valid_cmd)
	{
		cmd_vel.angular.z = v_theta_samp;
		return true;
	}

	cmd_vel.angular.z = 0.0;
	return false;

}

bool TrajectoryPlannerROS::setPlan(
		const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
	/**
	 * @brief  Set the plan that the controller is following
	 * @param orig_global_plan The plan to pass to the controller
	 * @return True if the plan was updated successfully, false otherwise
	 */
	if (!isInitialized())
	{
		ROS_ERROR(
				"This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	//reset the global plan
	global_plan_.clear();
	global_plan_ = orig_global_plan;

	//when we get a new plan, we also want to clear any latch we may have on goal tolerances
	xy_tolerance_latch_ = false;

	return true;
}

/** *********************************************************************************************************
 * XXX TrajectoryPlannerROS::computeVelocityCommands: compute the velocity command when move base ask for it
 *
 **************************************************************************************************************/
bool TrajectoryPlannerROS::computeVelocityCommands(
		geometry_msgs::Twist& cmd_vel)
{
	/**
	 * @brief  Given the current position, orientation, and velocity of the robot,
	 * compute velocity commands to send to the base
	 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
	 * @return True if a valid trajectory was found, false otherwise
	 */

	std::vector<geometry_msgs::PoseStamped> local_plan;
	std::vector<geometry_msgs::PoseStamped> partial_plan;//The first plan selected out of the list of all possible plans
	std::vector<geometry_msgs::PoseStamped> transformed_plan;//The global plan in the robot's base_link frame
	tf::Stamped<tf::Pose> global_pose; //The current global pose of the robot
	tf::Stamped<tf::Pose> drive_cmds; //The list of  poses of the best simulated trajectory (if there was a valid one).
	drive_cmds.frame_id_ = robot_base_frame_;
	tf::Stamped<tf::Pose> robot_vel; //The current robot velocity taken from robot's odometry
	tf::Stamped<tf::Pose> goal_point; //The goal point is the last point of the current path

	// XXX Check if everything was initialized correctly.
	//If planner was not initialized
	if (!isInitialized())
	{
		ROS_ERROR(
				"This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	// If the pose of the robot is not known
	if (!costmap_ros_->getRobotPose(global_pose))
	{
		return false;
	}
	//If the global path can not be transformed to local coordinates
	if (!transformGlobalPlan(*tf_, global_plan_, global_pose, costmap_,
			global_frame_, transformed_plan)) //get the global plan in our frame
	{
		ROS_WARN(
				"Could not transform the global plan to the frame of the controller");
		return false;
	}
	//now we'll prune the plan based on the position of the robot
	if (prune_plan_)
		prunePlan(global_pose, transformed_plan, global_plan_);
	//if the global plan passed in is empty... we won't do anything
	if (transformed_plan.empty())
		return false;

	//XXX Update the costmap we'll use for this cycle
	costmap_ros_->getCostmapCopy(costmap_);
	costmap_ros_->clearRobotFootprint(); //Clear the robot footprint from the costmap we're using

	//XXX Get robot's odometry
	odom_helper_.getRobotVel(robot_vel);

	//XXX Get current goal
	tf::poseStampedMsgToTF(transformed_plan.back(), goal_point); //we assume the global goal is the last point in the global plan
	double goal_x = goal_point.getOrigin().getX();
	double goal_y = goal_point.getOrigin().getY();
	double yaw = tf::getYaw(goal_point.getRotation());
	double goal_th = yaw;

	//XXX Set current global plan
	tc_->updatePlan(transformed_plan);
	/**
	 * XXX SET ui commands (new data) to the tc_ controller.
	 */
	geometry_msgs::Twist user_vel; //The current input command coming from by the user
//	geometry_msgs::Twist face_dir; //The current direction of the user's command
	std_msgs::String user_voice; //The last vocal command
	//if (user_vel_handler_.newInputAvailabe())//If the user gave a new command then we change the new value
	//{
	user_vel_handler_.read(user_vel);
	//user_vel_handler_.setNewInputFlag(false);
	tc_->setUserInput(user_vel.linear.x, user_vel.linear.y,
			user_vel.angular.z); //set,initialize the user's desired vel into the trajectory planner
	//}
//	face_dir_handler_.read(face_dir);
	voice_handler_.read(user_voice);
//	tc_->setFaceDir(face_dir.angular.x, face_dir.angular.y, face_dir.angular.z); //set,initialize the user's desired vel into the trajectory planner
	tc_->setUserVoice(user_voice);

	/*
	 *  SPECIAL BEHAVIOUR WHEN CLOSE TO THE GOAL
	 */
	if (xy_tolerance_latch_
			|| (getGoalPositionDistance(global_pose, goal_x, goal_y)
					<= xy_goal_tolerance_))
	{
		//if the user wants to latch goal tolerance, if we ever reach the goal xy_tolerance_latch_ location, we'll
		//just rotate in place
		if (latch_xy_goal_tolerance_)
		{
			xy_tolerance_latch_ = true;	//set this the first time the robot reaches the goal. so that if it goes out
			//while rotating to align it doesn't care.
		}

		double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
		//if the goal orientation has been reached then stop.
		if (fabs(angle) <= yaw_goal_tolerance_)
		{
			//set the velocity command to zero
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
			rotating_to_goal_ = false;
			xy_tolerance_latch_ = false;
		}				//if the goal orientation has been reached
		else //if the angle is still big
		{

			//XXX we need to call the next two lines to make sure that the trajectory
			//planner updates its path distance and goal distance grids
			tc_->updatePlan(transformed_plan);
			Trajectory path = tc_->findBestPathVector(global_pose, robot_vel,
					drive_cmds, evaluated_trajs_);
			map_viz_.publishCostCloud(costmap_);
			nav_msgs::Odometry base_odom; //copy over the odometry information
			odom_helper_.getOdom(base_odom);

			//if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
			if (!rotating_to_goal_
					&& !ui_local_planner::stopped(base_odom,
							rot_stopped_velocity_, trans_stopped_velocity_))
			{
				if (!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
				{
					return false;
				}
			}
			//if we're stopped... then we want to rotate to goal
			else
			{
				//set this so that we know its OK to be moving
				rotating_to_goal_ = true;
				if (!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
				{
					return false;
				}
			}
		} //else (if the goal orientation has been reached)

		//publish an empty plan because we've reached our goal position
		publishPlan(transformed_plan, g_plan_pub_);
		publishPlan(local_plan, l_plan_pub_);
		publishPlan(partial_plan, partial_plan_pub_);

		//we don't actually want to run the controller when we're just rotating to goal
		return true;
	} //finish if we've reached the goal position check

	/*********************************************************************
	 *******************THIS IS WHERE WE COMPUTE EVERYTHING ***************
	 **********************************************************************/
	//XXX compute the set of dynamic window trajectories
	//This is the best local path for the given robot speed, pose and desired velocity
	Trajectory path = tc_->findBestPathVector(global_pose, robot_vel,
			drive_cmds, evaluated_trajs_);
	/**********************************************************************/

	map_viz_.publishCostCloud(costmap_);

	//XXX if there is no valid trajectory tell someone //TODO:send user command if not valid plan was found.
	if (path.cost_ < 0)
	{
		ROS_DEBUG_NAMED("trajectory_planner_ros",
				"The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
		local_plan.clear();
		evaluated_trajs_.clear();
		publishPlan(transformed_plan, g_plan_pub_);
		publishPlan(local_plan, l_plan_pub_);
		publishPlan(partial_plan, partial_plan_pub_);
		return false;
	}

	//XXX If there is any good trajectory pass along drive commands
	cmd_vel.linear.x = drive_cmds.getOrigin().getX();
	cmd_vel.linear.y = drive_cmds.getOrigin().getY();
	cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());
	ROS_DEBUG_NAMED("trajectory_planner_ros",
			"A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
			cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

	//XXX Publish visualization dataRVIZ
	for (unsigned int i = 0; i < path.getPointsSize(); ++i)
	{
		double p_x, p_y, p_th;
		path.getPoint(i, p_x, p_y, p_th);
		tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(
				tf::Pose(tf::createQuaternionFromYaw(p_th),
						tf::Point(p_x, p_y, 0.0)), ros::Time::now(),
				global_frame_);
		geometry_msgs::PoseStamped pose;
		tf::poseStampedTFToMsg(p, pose);
		local_plan.push_back(pose);
	}
	Trajectory partial_path = evaluated_trajs_[0]; //The first element in the vector of evaluated trajectories is the best
	//To print all the evaluated paths (for visualization purposes) however it can be removed to improve performance
	if (publish_traj_markers_)
	{
		markers_handler_.write(evaluated_trajs_);
	}

	//publish information to the visualizer
	publishPlan(transformed_plan, g_plan_pub_);
	publishPlan(local_plan, l_plan_pub_);
	publishPlan(partial_plan, partial_plan_pub_);
	evaluated_trajs_.clear();
	return true;
}

double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp,
		double vtheta_samp, bool update_map)
{
	/**
	 * @brief  Generate and score a single trajectory
	 * @param vx_samp The x velocity used to seed the trajectory
	 * @param vy_samp The y velocity used to seed the trajectory
	 * @param vtheta_samp The theta velocity used to seed the trajectory
	 * @param update_map Whether or not to update the map for the planner
	 * when computing the legality of the trajectory, this is useful to set
	 * to false if you're going to be doing a lot of trajectory checking over
	 * a short period of time
	 * @return score of trajectory (double)
	 */
	// Copy of checkTrajectory that returns a score instead of True / False
	tf::Stamped<tf::Pose> global_pose;
	if (costmap_ros_->getRobotPose(global_pose))
	{
		if (update_map)
		{
			//we also want to clear the robot footprint from the costmap we're using
			costmap_ros_->clearRobotFootprint();

			//make sure to update the costmap we'll use for this cycle
			costmap_ros_->getCostmapCopy(costmap_);

			//we need to give the planner some sort of global plan, since we're only checking for legality
			//we'll just give the robots current position
			std::vector<geometry_msgs::PoseStamped> plan;
			geometry_msgs::PoseStamped pose_msg;
			tf::poseStampedTFToMsg(global_pose, pose_msg);
			plan.push_back(pose_msg);
			tc_->updatePlan(plan, true);
		}

		//copy over the odometry information
		nav_msgs::Odometry base_odom;
		{
			boost::recursive_mutex::scoped_lock lock(odom_lock_);
			base_odom = base_odom_;
		}
		return tc_->scoreTrajectory(global_pose.getOrigin().x(),
				global_pose.getOrigin().y(),
				tf::getYaw(global_pose.getRotation()),
				base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y,
				base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

	}
	ROS_WARN(
			"Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
	return -1.0;
}

bool TrajectoryPlannerROS::isGoalReached()
{
	/**
	 * @brief  Check if the goal pose has been achieved
	 * @return True if achieved, false otherwise
	 */
	if (!isInitialized())
	{
		ROS_ERROR(
				"This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	//copy over the odometry information
	nav_msgs::Odometry base_odom;
	odom_helper_.getOdom(base_odom);
	tf::Stamped<tf::Pose> global_pose;
	costmap_ros_->getRobotPose(global_pose);
	return ui_local_planner::isGoalReached(*tf_, global_plan_, costmap_,
			global_frame_, global_pose, base_odom, rot_stopped_velocity_,
			trans_stopped_velocity_, xy_goal_tolerance_, yaw_goal_tolerance_);
}

double TrajectoryPlannerROS::getTrajectory(double vx_samp, double vy_samp,
		double vtheta_samp, bool update_map, Trajectory & trajectory)
{
	ROS_DEBUG(
			"In  TrajectoryPlannerROS::getTrajectory vx_samp:%f, vtheta_samp:%f",
			vx_samp, vtheta_samp);
	tf::Stamped<tf::Pose> global_pose;
	if (costmap_ros_->getRobotPose(global_pose))
	{
		if (update_map)
		{
			//we also want to clear the robot footprint from the costmap we're using
			costmap_ros_->clearRobotFootprint();

			//make sure to update the costmap we'll use for this cycle
			costmap_ros_->getCostmapCopy(costmap_);

			//we need to give the planne some sort of global plan, since we're only checking for legality
			//we'll just give the robots current position
			std::vector<geometry_msgs::PoseStamped> plan;
			geometry_msgs::PoseStamped pose_msg;
			tf::poseStampedTFToMsg(global_pose, pose_msg);
			plan.push_back(pose_msg);
			tc_->updatePlan(plan, true);
		}

		//copy over the odometry information
		nav_msgs::Odometry base_odom;
		{
			boost::recursive_mutex::scoped_lock lock(odom_lock_);
			base_odom = base_odom_;
		}

		return tc_->getTrajectory(global_pose.getOrigin().x(),
				global_pose.getOrigin().y(),
				tf::getYaw(global_pose.getRotation()),
				base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y,
				base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp,
				trajectory);

	}
	ROS_WARN(
			"Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
	return (-1);
}

void TrajectoryPlannerROS::set_escape_mode(double escape_vel)
{
	// Copy of checkTrajectory that returns a score instead of True / False
	tf::Stamped<tf::Pose> global_pose;
	if (costmap_ros_->getRobotPose(global_pose))
	{
		tc_->setEscapeMode(global_pose.getOrigin().getX(),
				global_pose.getOrigin().getY(),
				tf::getYaw(global_pose.getRotation()), escape_vel);
	}
	else
	{
		ROS_WARN(
				"Failed to get the pose of the robot. When trying to set the escape mode");
	}

}
}
;
