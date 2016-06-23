/*********************************************************************
 * Author:Arturo Escobedo
 *********************************************************************/
#ifndef USER_DIR_RECOVERY_USER_DIR_RECOVERY_H
#define USER_DIR_RECOVERY_USER_DIR_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

namespace user_dir_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.
class UserDirRecovery: public nav_core::RecoveryBehavior
{
public:

	/// Doesn't do anything: initialize is where the actual work happens
	UserDirRecovery();

	~UserDirRecovery();

	/// Initialize the parameters of the behavior
	void initialize(std::string n, tf::TransformListener* tf,
			costmap_2d::Costmap2DROS* global_costmap,
			costmap_2d::Costmap2DROS* local_costmap);

	/// Run the behavior
	void runBehavior();

private:

	geometry_msgs::Pose2D getCurrentLocalPose() const;
	geometry_msgs::Twist scaleGivenAccelerationLimits(
			const geometry_msgs::Twist& twist,
			const double time_remaining) const;
	double nonincreasingCostInterval(const geometry_msgs::Pose2D& current,
			const geometry_msgs::Twist& twist) const;
	double normalizedPoseCost(const geometry_msgs::Pose2D& pose) const;
	geometry_msgs::Twist transformTwist(
			const geometry_msgs::Pose2D& pose) const;

	ros::NodeHandle nh_;
	costmap_2d::Costmap2DROS* global_costmap_;
	costmap_2d::Costmap2DROS* local_costmap_;
	costmap_2d::Costmap2D costmap_; // Copy of local_costmap_, used by world model
	std::string name_;
	tf::TransformListener* tf_;
	ros::Publisher pub_, wheelchair_talk_pub_;
	ros::Subscriber input_sub_;
	ros::Subscriber input_voice_sub_;
	bool initialized_;
	void _input_vel_callback_(const geometry_msgs::TwistConstPtr& vel);
	void _input_voice_callback_(const std_msgs::StringConstPtr& voice);
	void ask_for_help(void);
	// Memory owned by this object
	// Mutable because footprintCost is not declared const
	mutable base_local_planner::CostmapModel* world_model_;

	geometry_msgs::Twist base_frame_twist_;

	double duration_;
	double linear_speed_limit_;
	double angular_speed_limit_;
	double linear_acceleration_limit_;
	double angular_acceleration_limit_;
	double controller_frequency_;
	double simulation_inc_;
	bool received_move_flag_;
	geometry_msgs::Twist input_vel_;
	std_msgs::String input_voice_;
	ros::ServiceClient destination_inference_enable_client_; //A Service client to activate the destination inference module
	ros::ServiceClient destination_inference_disable_client_; //A Service client to deactivate the destination inference module

};

} // namespace user_dir_recovery

#endif // include guard
