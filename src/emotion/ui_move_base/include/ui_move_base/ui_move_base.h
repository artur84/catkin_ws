#ifndef NAV_UI_MOVE_BASE_ACTION_H_
#define NAV_UI_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ui_local_planner/trajectory_planner_ros.h>
#include <user_intentions/LocalGoalArray.h>
#include <user_intentions/LocalGoal.h>
#include <pal_msgs/PoseWithVelocity.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "ui_move_base/UIMoveBaseConfig.h"
#include <std_srvs/Empty.h>
#include <controller/EnableControl.h>

namespace ui_move_base
{
//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

enum UIMoveBaseState
{
	PLANNING, CONTROLLING, CLEARING,
};

enum RecoveryTrigger
{
	PLANNING_R, CONTROLLING_R, OSCILLATION_R
};

enum UINavigationBehavior
{
	AUTONOMOUS, //Wheelchair moving according to the ROS navigation system
	FOLLOW, 	//The wheelchair is following a person
	FACE,  		//The wheelchair is operated using commands from the user

};

enum UINavSubBehaviour
{
	BACK,       //Moving back enabled by the user
	TURN, 		//Turning enabled by the user
	FORWARD,     //Moving forward
	BRAKE		//Stopped
};

enum WheelchairMode
{
	COMPUTER, //wheelchair receiving commands from computer
	JOYSTICK  //wheelchair using joystick
};

/**
 * @class UIMoveBase
 * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
 */
class UIMoveBase
{
public:

	UIMoveBase(std::string name, tf::TransformListener& tf);

	virtual ~UIMoveBase();

	bool executeCycle(geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& global_plan);

private:
	bool clearUnknownService(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &resp);

	bool clearCostmapsService(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &resp);

	bool planService(nav_msgs::GetPlan::Request &req,
			nav_msgs::GetPlan::Response &resp);

	bool makePlan(const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan);

	bool loadRecoveryBehaviors(ros::NodeHandle node);

	void loadDefaultRecoveryBehaviors();

	void clearCostmapWindows(double size_x, double size_y);

	void publishZeroVelocity();

	void resetState();

	void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

	void planThread();

	void executeCb(
			const move_base_msgs::MoveBaseGoalConstPtr& ui_move_base_goal);

	bool isQuaternionValid(const geometry_msgs::Quaternion& q);

	double distance(const geometry_msgs::PoseStamped& p1,
			const geometry_msgs::PoseStamped& p2);

	geometry_msgs::PoseStamped goalToGlobalFrame(
			const geometry_msgs::PoseStamped& goal_pose_msg);

	tf::TransformListener& tf_;

	MoveBaseActionServer* as_; //The move base action server

	costmap_2d::Costmap2DROS* global_planner_costmap_ros_;
	costmap_2d::Costmap2DROS controller_costmap_ros_;

	nav_core::BaseGlobalPlanner* global_planner_; //The A* global planner
	std::string robot_base_frame_, global_frame_;

	std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
	unsigned int recovery_index_;

	tf::Stamped<tf::Pose> global_pose_;
	double global_planner_frequency_, controller_frequency_, inscribed_radius_,
			circumscribed_radius_;
	double global_planner_patience_, controller_patience_;
	double conservative_reset_dist_, clearing_radius_;
	ros::Publisher current_goal_pub_, cmd_vel_pub_, action_goal_pub_;
	ros::Subscriber goal_sub_;
	ros::ServiceServer make_plan_srv_, clear_unknown_srv_, clear_costmaps_srv_;
	bool shutdown_costmaps_, clearing_roatation_allowed_,
			recovery_behavior_enabled_;
	double oscillation_timeout_, oscillation_distance_;

	UIMoveBaseState movebase_state_;
	RecoveryTrigger recovery_trigger_;
	UINavigationBehavior navigation_behavior_;
	UINavSubBehaviour nav_sub_behavior_;
	WheelchairMode wheelchair_mode_;

	ros::Time last_valid_plan_time_, last_valid_control_time_,
			last_oscillation_reset_time_;
	geometry_msgs::PoseStamped oscillation_pose_;
	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
	pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

	//set up plan triple buffer
	std::vector<geometry_msgs::PoseStamped>* global_planner_plan_;
	std::vector<geometry_msgs::PoseStamped>* latest_plan_;
	std::vector<geometry_msgs::PoseStamped>* controller_plan_;

	//set up the planner's thread
	bool runPlanner_;
	boost::mutex global_planner_mutex_;
	boost::condition_variable global_planner_cond_;
	geometry_msgs::PoseStamped global_planner_goal_;
	boost::thread* global_planner_thread_;

	boost::recursive_mutex configuration_mutex_;
	dynamic_reconfigure::Server<ui_move_base::UIMoveBaseConfig> *dsrv_;

	void reconfigureCB(ui_move_base::UIMoveBaseConfig &config, uint32_t level);

	ui_move_base::UIMoveBaseConfig last_config_;
	ui_move_base::UIMoveBaseConfig default_config_;
	bool setup_, p_freq_change_, c_freq_change_;
	bool new_global_plan_;

	//Voice interface
	void input_voice_callback_(const std_msgs::String::ConstPtr & msg);
	ros::Subscriber input_voice_sub_; //The ROS subscriber to the voice command given by the user
	std_msgs::String input_voice_;

	//User intentions
	bool correct_user_vel_();
	void input_vel_callback_(const geometry_msgs::TwistConstPtr& vel);
	ui_local_planner::MarkersHandler markers_handler_; //Needed to display the produced trajectories
	std::vector<ui_local_planner::Trajectory> trajectories_vect; //Needed to display the produced trajectories
	ui_local_planner::TrajectoryPlannerROS tc_ros_; //A ROS wrapper of the class that Implements the dynamic window approach
	boost::mutex mutex_; //To control access to cmd_vel message
	geometry_msgs::Twist input_vel_; //The input (user's desired velocity).
	ui_local_planner::Trajectory trajectory_; //The simulated trajectory given the current position of the robot, and (v,w)
	boost::thread* local_planning_thread_; //boost thread for dynamic windows planning
	double theta_range_;
	int num_th_samples_, num_x_samples_;
	double collision_trans_speed_, collision_rot_speed_;
	std::string tf_prefix_;
	ros::Subscriber input_vel_sub_;
	ros::Subscriber target_person_pos_sub_;
	double occ_cost_scale_, user_vel_cost_scale_; //How important is dynamic_window cost and user_vel in the total_cost_function

	//Goals destination_inference
	void goal_array_callback_(
			const user_intentions::LocalGoalArrayConstPtr& goals_msg); //To receive the list of typical destinations
																	   //published by the destination inference node
	void target_person_pos_callback_(
			const pal_msgs::PoseWithVelocityConstPtr& msg);
	void publish_paths(std::vector<nav_msgs::Path>& paths);
	void transform_pose_vector_to_path(
			const std::vector<geometry_msgs::PoseStamped> & vector,
			nav_msgs::Path & path);
	bool get_many_paths(user_intentions::LocalGoalArray & goal_array,
			std::vector<std::vector<geometry_msgs::PoseStamped> >& plan_vector);

	user_intentions::LocalGoalArray goal_array_;
	user_intentions::LocalGoal goal1_;
	bool print_multiple_paths_; //Flag to indicate if it should or not print multiple paths
	int num_paths_; //Flag to indicate if it should or not print multiple paths
	bool publish_traj_markers_; //Publish the arcs of the dynamic window simulated trajectories
	bool try_user_vel_first_; //Flag to indicate if we send the input from the user directly when receiving it in the callback
	double go_time_; //how much time to move forward after a go command (seconds)
	ros::Subscriber goal_array_sub_; //The ROS subscriber to the list of goals published by the destination inference module.
	ros::Publisher paths_pub_;
	pal_msgs::PoseWithVelocity target_person_pos_; //A message containing both relative pose and velocity of the target with respect to the robot

	//Mode selector
	ros::ServiceClient wheelchair_auto_client_; //A Service client to put the wheelchair in autnomouos mode
	ros::ServiceClient wheelchair_manual_client_; //A Service client to put the wheelchair in manual mode
	ros::ServiceClient follower_enable_client_; //A Service client to activate/deactivate the follower

	ros::Time input_voice_time_; //The time when the last vocal command was received

};
}
;
#endif

