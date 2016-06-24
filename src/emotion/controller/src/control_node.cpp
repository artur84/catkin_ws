#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pal_msgs/PoseWithVelocity.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <control/TrackingController.h>
#include <control/TrackingControllerPositionParametric.h>
#include <control/TrackingControllerSingularYAxis.h>
#include <control/TrackingControllerSingularYAxisPos.h>
#include <control/TrackingControllerVelocity.h>
//#include <control/TrackingControllerLinearPosition.h>
//#include <control/TrackingControllerLinearFullState.h>

#include <dynamic_reconfigure/server.h>
#include <controller/controller_paramsConfig.h>
#include <controller/EnableControl.h>

#include <control/ctrl_traits.h>

pal_msgs::PoseWithVelocity m_robotPose;
boost::mutex m_usingRobotPose;
boost::shared_ptr<TrackingController> m_ctrl;
ros::Publisher m_pub_vel;
ros::Publisher g_pub_xr;
ros::Publisher g_pub_pfm;
ros::Publisher g_pub_debug;
ros::ServiceServer g_srv_control;

std::vector<double> g_k(SIZE_GAIN, 0);
std::vector<double> g_Pfm(SIZE_PFM, 0);
std::vector<double> g_xr_thr(SIZE_XR_THR, 0);
std::vector<double> g_xrD(SIZE_XRD, 0);
std::vector<double> g_xm_thm(SIZE_XM_THM, 0);
std::string g_ctrl_laws = "parametric";
double g_last_seen_time            = 0;
const double TIME_TO_FAILSAFE_STOP = 1;
const double TIME_TO_LOOKUP = 0.2;
boost::shared_ptr<tf::TransformListener> tfListener;
bool g_enable_debug = false;
bool g_control_enabled = false;
std::string g_debug_frame_id = "base_link";
std::vector<double> P_r_D(2,0);
std::vector<double> P_r(2,0);
geometry_msgs::Twist g_ctrlCmd;
geometry_msgs::Twist g_ctrlCmd_lastValid;
geometry_msgs::PoseStamped g_old_pose;

enum VisDebug
{
	DBG_TGT_POSE_BODY = 0,
	DBG_TGT_POSE_HEAD
};


bool selectCtrlLaws(const std::string& ctrl_laws)
{
	ROS_INFO("Selecting %s", ctrl_laws.c_str());
	if (g_ctrl_laws != ctrl_laws || m_ctrl.get() == NULL)
	{
		if (m_ctrl.get() != NULL)
		{
			m_ctrl.reset();
		}
		if (ctrl_laws == "parametric")
		{
			ROS_INFO("parametric selected");
			m_ctrl = boost::shared_ptr<TrackingController>(new TrackingControllerPositionParametric());
		}
		else if (ctrl_laws == "singular")
		{
			ROS_INFO("singular selected");
			m_ctrl = boost::shared_ptr<TrackingController>(new TrackingControllerSingularYAxis());
		}
		else if (ctrl_laws == "singular_pos")
		{
			ROS_INFO("singular+position selected");
			m_ctrl = boost::shared_ptr<TrackingController>(new TrackingControllerSingularYAxisPos());
		}
		else if (ctrl_laws == "velocity")
		{
			ROS_INFO("velocity selected");
			m_ctrl = boost::shared_ptr<TrackingController>(new TrackingControllerVelocity());
		}
		else
		{
			ROS_ERROR("Invalid control laws selected: '%s', aborting...", ctrl_laws.c_str());
			return false;
		}
	}
	g_ctrl_laws = ctrl_laws;
	return true;
}

void configCallback(controller::controller_paramsConfig &config, uint32_t level)
{
	g_k[0]	 = config.k1;
	g_k[1]	 = config.k2;
	g_k[2]	 = config.k3;
	g_k[3]	 = config.k4;
	g_Pfm[0] = config.Pfm_x;
	g_Pfm[1] = config.Pfm_y;

	selectCtrlLaws(config.ctrl_laws);
	m_ctrl->setGain(g_k);
	m_ctrl->setPfm(g_Pfm);
	geometry_msgs::Point pfm;
	pfm.x = g_Pfm[0];
	pfm.y = g_Pfm[1];
	g_pub_pfm.publish(pfm);
	g_enable_debug = config.enable_debug;
}

/*
 * Callback for target pose messages
 * Target pose *should* be w.r.t. the mobile frame
 *
 */
double g_th_last = 0;
void targetPoseCallback(const pal_msgs::PoseWithVelocity::ConstPtr& msg)
{
	g_ctrlCmd.linear.x  = 0;
	g_ctrlCmd.angular.z = 0;
	if (msg->pose.x == 0 && msg->pose.y == 0)
		return;
	{
		boost::mutex::scoped_lock lock(m_usingRobotPose);
		// setting current robot pose
		g_xm_thm[0] = m_robotPose.pose.x;
		g_xm_thm[1] = m_robotPose.pose.y;
		g_xm_thm[2] = m_robotPose.pose.theta;
		// WARNING:
		// m_robotPose.vx is the "linear" velocity (not vx component)
		// m_robotPose.vy is the "angular" velocity (not vy component)
		g_xm_thm[3] = m_robotPose.vx;
		g_xm_thm[4] = m_robotPose.vy;
		m_ctrl->setRobotPose(g_xm_thm);

		ros::Time now = ros::Time::now();

		geometry_msgs::PoseStamped ps,psout;
		ps.pose.position.x   = msg->pose.x;
		ps.pose.position.y   = msg->pose.y;
		ps.pose.orientation  = tf::createQuaternionMsgFromYaw(msg->pose.theta);
		ps.header = msg->header;
		tf::Stamped<tf::Transform> transform;
		try
		{
			tfListener->waitForTransform(msg->header.frame_id, g_debug_frame_id, now, ros::Duration(TIME_TO_LOOKUP));
			tfListener->transformPose(g_debug_frame_id, ps, psout);
		}
		catch(tf::ExtrapolationException e)
		{
			ROS_WARN("tf::ExtrapolationException (but continuing anyway...): %s", e.what());
			psout = g_old_pose;
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("Skipping control cycle because of: %s", e.what());
			return;
		}

		// setting target pose
		// P_r   := target position w.r.t. the mobile frame
		// P_r_D := derivative of P_r
		P_r[0] = psout.pose.position.x;
		P_r[1] = psout.pose.position.y;
		P_r_D[0] = msg->vx;
		P_r_D[1] = msg->vy;
		/* x_r = x_m + R*P_r
		 *
		 *       | x_m_1 |   | cos(th_m) -sin(th_m) | | P_r_1 |
		 * x_r = |       | + |                      | |       |
		 *       | x_m_2 |   | sin(th_m)  cos(th_m) | | P_r_2 |
		 */
		//double cth  = cos(g_xm_thm[2]),
		//       sth         = sin(g_xm_thm[2]);
		g_xr_thr[0] = P_r[0]; //g_xm_thm[0] + cth*P_r[0] - sth*P_r[1];
		g_xr_thr[1] = P_r[1]; //g_xm_thm[1] + sth*P_r[0] + cth*P_r[1];
		// NB we do not know target orientation when in mobile frame,
		// so we use the direction of velocity
		g_xr_thr[2] = 0; //atan2(g_xr_thr[1], g_xr_thr[0]);

		// setting target velocity
		/* \dot{x_r} = v_m*R*e_1 + \omega*S*R*P_r + R*\dot{P_r}
		 * v_r = R'*\dot{x_r} = v_m*e_1 + \omega*S*P_r + \dot{P_r}
		 *
		 *           | 1 |          | 0  -1 || P_r_1 |   | P_r_D_1 |
		 * v_r = v_m |   | + \omega |       ||       | + |         |
		 *           | 0 |          | 1   0 || P_r_2 |   | P_r_D_2 |
		 */
		g_xrD[0] =  P_r_D[0]; //v_m - w_m*P_r[1] + P_r_D[0];
		g_xrD[1] =  P_r_D[1]; //      w_m*P_r[0] + P_r_D[1];
		g_old_pose = psout;

		m_ctrl->setTargetPose(P_r);
		m_ctrl->setTargetVelocity(P_r_D);
		m_ctrl->setRobotPose(g_xm_thm);
		geometry_msgs::Pose xr;
		xr.position.x  = g_xr_thr[0];
		xr.position.y  = g_xr_thr[1];
		xr.orientation = tf::createQuaternionMsgFromYaw(g_xr_thr[2]);
		g_pub_xr.publish(xr);
		geometry_msgs::Point pfm;
		pfm.x = g_Pfm[0];
		pfm.y = g_Pfm[1];
		g_pub_pfm.publish(pfm);

		// doing control
		m_ctrl->do_control(g_ctrlCmd.linear.x, g_ctrlCmd.angular.z);

		// publishing info
		if (g_control_enabled)
		{
			if (fabs(g_ctrlCmd.linear.x) < 0.01)
				g_ctrlCmd.linear.x = 0;
			if (fabs(g_ctrlCmd.angular.z) < 0.01)
				g_ctrlCmd.angular.z = 0;
			m_pub_vel.publish(g_ctrlCmd);
		}
		g_last_seen_time = ros::Time::now().toSec();

		if (g_enable_debug)
		{
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray marker_array;
			marker.header.frame_id = g_debug_frame_id;
			marker.header.stamp = now;
			marker.ns = "ctrl";
			marker.lifetime = ros::Duration(0.1);
			marker.id = DBG_TGT_POSE_BODY;
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = P_r[0]; //g_xr_thr[0];
			marker.pose.position.y = P_r[1]; //g_xr_thr[1];
			marker.pose.position.z = 0.75;
			//marker.pose.orientation = tf::createQuaternionMsgFromYaw(g_xr_thr[2]);
			marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 1.5;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker_array.markers.push_back(marker);
			marker.type            = visualization_msgs::Marker::SPHERE;
			marker.id = DBG_TGT_POSE_HEAD;
			marker.pose.position.z = 1.6;
			marker.scale.x         = marker.scale.y                      = marker.scale.z = 0.3;
			marker_array.markers.push_back(marker);
			g_pub_debug.publish(marker_array);
		}
	}
	//ROS_INFO("last exe of control_node was %f sec", ros::Time::now().toSec() - now);
}

void timerCallback(const ros::TimerEvent&)
{
	double delta_time = ros::Time::now().toSec() - g_last_seen_time;
	if (delta_time > TIME_TO_FAILSAFE_STOP)
	{
		// stopping since we lost target (or not received information since long
		// time)
		//ROS_WARN("Target is lost (lastseen was %f secs ago), stopping ctrl",delta_time);
		g_ctrlCmd.linear.x  = 0;
		g_ctrlCmd.angular.z = 0;
		m_pub_vel.publish(g_ctrlCmd);
		//ROS_INFO("Publishing ctrl %f %f", g_ctrlCmd.linear.x, g_ctrlCmd.angular.z);
	}
}

/*
 * Callback for robot pose messages
 *
 */
void robotPoseCallback(const pal_msgs::PoseWithVelocity::ConstPtr& msg)
{
	{
		// trying to lock: if not succeded, robotPose is currently used by targetPoseCallback
		boost::mutex::scoped_try_lock lock(m_usingRobotPose);
		if ( ! lock.owns_lock())
		{
			// doing nothing: skipping message
			ROS_WARN("Lock found in robotPoseCallback");
			return;
		}
		// updating robot pose
		m_robotPose = *(msg.get());
		//ROS_INFO("Get robotPose: %f %f %f", m_robotPose.pose.x, m_robotPose.pose.y, m_robotPose.pose.theta);
	}
}

bool enable_control_cb(
	controller::EnableControl::Request& req, 
	controller::EnableControl::Response& res)
{
	g_control_enabled = req.enabled;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_node");
	ros::NodeHandle n;

	tfListener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(TIME_TO_FAILSAFE_STOP)));

	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("k1", g_k[0], double(g_k[0]));
	private_node_handle_.param("k2", g_k[1], double(g_k[1]));
	private_node_handle_.param("k3", g_k[2], double(g_k[2]));
	private_node_handle_.param("k4", g_k[3], double(g_k[3]));
	private_node_handle_.param("Pfm_x", g_Pfm[0], double(g_Pfm[0]));
	private_node_handle_.param("Pfm_y", g_Pfm[1], double(g_Pfm[1]));
	private_node_handle_.param("ctrl_laws", g_ctrl_laws, g_ctrl_laws);
	private_node_handle_.param("enable_debug", g_enable_debug, g_enable_debug);
	private_node_handle_.param("debug_frame_id", g_debug_frame_id, g_debug_frame_id);

	// initialising controller
	if (!selectCtrlLaws(g_ctrl_laws))
		return -1;

	// initialising input/output topics
	m_pub_vel = n.advertise<geometry_msgs::Twist>("robot_ctrl", 1);
	g_pub_xr  = private_node_handle_.advertise<geometry_msgs::Pose>("xr", 1);
	g_pub_pfm = private_node_handle_.advertise<geometry_msgs::Point>("Pfm", 1);
	g_pub_debug = private_node_handle_.advertise<visualization_msgs::MarkerArray>("visual_debug",0);
	ros::Subscriber sub_robot_pose  = private_node_handle_.subscribe("robot_pose", 1, robotPoseCallback);
	ros::Subscriber sub_target_pose  = private_node_handle_.subscribe("target_pose", 1, targetPoseCallback);

	// Set up a dynamic reconfigure server.
	// This should be done before reading parameter server values. 
	dynamic_reconfigure::Server<controller::controller_paramsConfig> dr_srv;
	dynamic_reconfigure::Server<controller::controller_paramsConfig>::CallbackType cb;
	cb = boost::bind(&configCallback, _1, _2);
	dr_srv.setCallback(cb);

	g_srv_control = private_node_handle_.advertiseService("enable_control", enable_control_cb);


	geometry_msgs::Point pfm;
	pfm.x = g_Pfm[0];
	pfm.y = g_Pfm[1];
	g_pub_pfm.publish(pfm);

	double timer_rate = 20; // should be lower than producer
	ros::Timer timer = n.createTimer(ros::Duration(1/timer_rate), timerCallback);

	ros::spin();

	return 0;
}

