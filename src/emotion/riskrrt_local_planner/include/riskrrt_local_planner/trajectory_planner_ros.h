#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <riskrrt_local_planner/world_model.h>
#include <riskrrt_local_planner/point_grid.h>
#include <riskrrt_local_planner/costmap_model.h>
#include <riskrrt_local_planner/voxel_grid_model.h>
#include <riskrrt_local_planner/map_grid_visualizer.h>

#include <riskrrt_local_planner/planar_laser_scan.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <riskrrt_local_planner/RISKRRTLocalPlannerConfig.h>

#include <riskrrt_local_planner/odometry_helper_ros.h>
#include <riskrrt_local_planner/user_vel_handler.h>

namespace riskrrt_local_planner
{
/**
 * @class TrajectoryPlannerROS
 * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
 */
class TrajectoryPlannerROS: public nav_core::BaseLocalPlanner
{
public:
	/**
	 * @brief  Default constructor for the ros wrapper
	 */
	TrajectoryPlannerROS();

	/**
	 * @brief  Constructs the ros wrapper
	 * @param name The name to give this instance of the trajectory planner
	 * @param tf A pointer to a transform listener
	 * @param costmap The cost map to use for assigning costs to trajectories
	 */
	TrajectoryPlannerROS(std::string name, tf::TransformListener* tf,
			costmap_2d::Costmap2DROS* costmap_ros);

	/**
	 * @brief  Constructs the ros wrapper
	 * @param name The name to give this instance of the trajectory planner
	 * @param tf A pointer to a transform listener
	 * @param costmap The cost map to use for assigning costs to trajectories
	 */
	void initialize(std::string name, tf::TransformListener* tf,
			costmap_2d::Costmap2DROS* costmap_ros);

	/**
	 * @brief  Destructor for the wrapper
	 */
	~TrajectoryPlannerROS();

	/**
	 * @brief  Given the current position, orientation, and velocity of the robot,
	 * compute velocity commands to send to the base
	 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
	 * @return True if a valid trajectory was found, false otherwise
	 */
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

	/**
	 * @brief  Set the plan that the controller is following
	 * @param orig_global_plan The plan to pass to the controller
	 * @return True if the plan was updated successfully, false otherwise
	 */
	bool setPlan(
			const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

	/**
	 * @brief  Check if the goal pose has been achieved
	 * @return True if achieved, false otherwise
	 */
	bool isGoalReached();

	/**
	 * @brief  Generate and score a single trajectory
	 * @param vx_samp The x velocity used to seed the trajectory
	 * @param vy_samp The y velocity used to seed the trajectory
	 * @param vtheta_samp The theta velocity used to seed the trajectory
	 * @param update_map Whether or not to update the map for the planner
	 * when computing the legality of the trajectory, this is useful to set
	 * to false if you're going to be doing a lot of trajectory checking over
	 * a short period of time
	 * @return True if the trajectory is legal, false otherwise
	 */
	bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp,
			bool update_map = true);

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
	double scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp,
			bool update_map = true);

	bool isInitialized()
	{
		return initialized_;
	}

private:
	/**
	 * @brief Callback to update the local planner's parameters based on dynamic reconfigure
	 */
	void reconfigureCB(RISKRRTLocalPlannerConfig &config, uint32_t level);

	/**
	 * @brief Once a goal position is reached... rotate to the goal orientation
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  goal_th The desired th value for the goal
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose,
			const tf::Stamped<tf::Pose>& robot_vel, double goal_th,
			geometry_msgs::Twist& cmd_vel);

	/**
	 * @brief Stop the robot taking into account acceleration limits
	 * @param  global_pose The pose of the robot in the global frame
	 * @param  robot_vel The velocity of the robot
	 * @param  cmd_vel The velocity commands to be filled
	 * @return  True if a valid trajectory was found, false otherwise
	 */
	bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose,
			const tf::Stamped<tf::Pose>& robot_vel,
			geometry_msgs::Twist& cmd_vel);

	/**
	 * @brief Fill the riskrrt LocalOccupancyGrid map
	 * @param  LocalMap& map
	 */
	void set_riskrrt_map();

	std::vector<double> loadYVels(ros::NodeHandle node);

	double sign(double x)
	{
		return x < 0.0 ? -1.0 : 1.0;
	}

	WorldModel* world_model_; ///< @brief The world model that the controller will use
	TrajectoryPlanner* tc_; ///< @brief The trajectory controller
	std::vector<Trajectory> evaluated_trajs_; //Vector of all the evaluated trajectories for visualizing
	costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
	costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use
	MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
	tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
	std::string global_frame_; ///< @brief The frame in which the controller will run
	double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
	nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
	std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
	double rot_stopped_velocity_, trans_stopped_velocity_;
	double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
	double inflation_radius_;
	std::vector<geometry_msgs::PoseStamped> global_plan_;
	bool prune_plan_;
	boost::recursive_mutex odom_lock_;

	double max_vel_th_, min_vel_th_;
	double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
	double sim_period_;
	bool rotating_to_goal_;
	bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
	geometry_msgs::Twist user_vel_; ///< @brief Used to get the velocity required by the user
	ros::Publisher g_plan_pub_, l_plan_pub_;
	ros::Publisher goal_marker_pub_; //Publisher to display a marker in a given goal position.
	ros::Publisher partial_plan_pub_;
	dynamic_reconfigure::Server<RISKRRTLocalPlannerConfig> *dsrv_;
	riskrrt_local_planner::RISKRRTLocalPlannerConfig default_config_;
	bool setup_;

	bool initialized_;
	riskrrt_local_planner::OdometryHelperRos odom_helper_;
	riskrrt_local_planner::UserVelHandler user_vel_handler_;

	/*
	 * Parameters for the riskrrt planner
	 */
//	double pThreshold_; /// @brief Threshold for the probability of collision: configurations with bigger P are discarded
//	bool p_space_scoring_; /// @brief If we want to respect personal space of humans around the robot
//	bool o_space_scoring_; /// @brief to take into account o-space (interaction space).
//	bool GP_predict_; /// @brief If it is True we use Gaussian Processes as prediction, if not, we just use linear prediction.
//	std::string dataDirGP_; /// @brief Database for typical trajectories used for GP computing
//	int loopRate_; /// @brief rate in hz of the loop of the planning (int) 1 Hz doesn't work
//	double pGoal_; /// @brief Probability of bias to the goal during the Growing.
	struct dataParam riskrrt_params_; 			/// @brief An structure containning some parameters to initialize riskrrtplanner
	std::string riskrrt_robot_config_dir_; 		/// @brief file from which we can read the robot configuration
	Wheelchair riskrrt_robot_; 					/// @brief The robot_object used by riskrrt
	RRT riskrrt_; 			///@brief The riskrrt planner
	LocalOccupancyGrid  riskrrt_occ_grid_; 		///	@brief The local occupancy grid used by riskrrt
	//TODO: Do I need this?? -> LocalMapHandlerRos ros_map_;				///	@brief ROS occupancy grid handler
	HumanHandlerRos hum_detected_;
	IntHandlerRos int_detected_;
	TreeRos ros_tree_;
	std::vector <PedestrianPredictor*> ped_predict_vect_; ///@brief Pedestrian predictor vector
	std::vector <double> ped_predict_time_vect_;///	@brief time vector for prediction
	Loader* gp_load_; 					///@brief This is to load the file with learned GP
	Pos riskrrt_goal_;
	State riskrrt_robot_origin_;
	Trajectory riskrrt_path_;
	RobotRos ros_robot_; 			/// @brief The robot_object used by riskrrt
	timeval timeLoc_; 				/// @brief TODO: what is this?
	timeval timeStepv_;				/// @brief TODO: what is this?
	pred_marker ped_predict_marker_;///	@brief pedestrian predictor markers for rviz

};
}
;
#endif
