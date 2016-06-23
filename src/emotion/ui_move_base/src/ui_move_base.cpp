#include <ui_move_base/ui_move_base.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace ui_move_base
{

UIMoveBase::UIMoveBase(std::string name, tf::TransformListener& tf) :
		tf_(tf), as_(NULL), global_planner_costmap_ros_(NULL), controller_costmap_ros_(
				"local_costmap", tf_), global_planner_(NULL), bgp_loader_(
				"nav_core", "nav_core::BaseGlobalPlanner"), recovery_loader_(
				"nav_core", "nav_core::RecoveryBehavior"), global_planner_plan_(
		NULL), latest_plan_(NULL), controller_plan_(NULL), runPlanner_(false), setup_(
				false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(
				false)
{
	/**
	 * @brief  Constructor for the actions
	 * @param name The name of the action
	 * @param tf A reference to a TransformListener
	 */
	as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base",
			boost::bind(&UIMoveBase::executeCb, this, _1), false);

	ros::NodeHandle private_nh("~");
	ros::NodeHandle nh;

	recovery_trigger_ = PLANNING_R;

	//get some parameters that will be global to the move base node
	std::string global_planner;
	private_nh.param("base_global_planner", global_planner,
			std::string("navfn/NavfnROS"));
	private_nh.param("global_costmap/robot_base_frame", robot_base_frame_,
			std::string("base_link"));
	private_nh.param("global_costmap/global_frame", global_frame_,
			std::string("/map"));
	private_nh.param("planner_frequency", global_planner_frequency_, 0.0);
	private_nh.param("controller_frequency", controller_frequency_, 20.0);
	private_nh.param("planner_patience", global_planner_patience_, 5.0);
	private_nh.param("controller_patience", controller_patience_, 15.0);

	private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
	private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
	private_nh.param("num_th_samples", num_th_samples_, 20);
	private_nh.param("num_x_samples", num_x_samples_, 10);
	private_nh.param("theta_range", theta_range_, 4.0);
	private_nh.param("translational_collision_speed", collision_trans_speed_,
			0.0);
	private_nh.param("rotational_collision__speed", collision_rot_speed_, 0.0);
	private_nh.param("tf_prefix", tf_prefix_, std::string(""));
	private_nh.param("occ_cost_scale", occ_cost_scale_, 0.3);
	private_nh.param("user_vel_cost_scale", user_vel_cost_scale_, 0.1);
	private_nh.param("print_multiple_paths", print_multiple_paths_, true);
	private_nh.param("num_paths", num_paths_, 3);
	private_nh.param("try_user_vel_first", try_user_vel_first_, true);
	private_nh.param("publish_traj_markers", publish_traj_markers_, false);

	if (!tf_prefix_.empty())
		tf_prefix_ = "/" + tf_prefix_;

	std::string global_frame = tf_prefix_ + "/map";

	//set up plan triple buffer
	global_planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
	latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
	controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

	//set up the planner's thread
	global_planner_thread_ = new boost::thread(
			boost::bind(&UIMoveBase::planThread, this));

	//for comanding the base
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>(
			"current_goal", 0);
	paths_pub_ = private_nh.advertise<nav_msgs::Path>("ui_path", 10);
	//Reading user's input
	input_vel_sub_ = nh.subscribe("user_vel", 1,
			&UIMoveBase::input_vel_callback_, this);
	target_person_pos_sub_ = nh.subscribe("target_person_pose_with_velocity", 1,
			&UIMoveBase::target_person_pos_callback_, this);
	input_voice_sub_ = nh.subscribe("recognizer/output", 1,
			&UIMoveBase::input_voice_callback_, this);
	goal_array_sub_ = nh.subscribe("goal_array", 1,
			&UIMoveBase::goal_array_callback_, this);
	ros::NodeHandle action_nh("move_base");
	action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>(
			"goal", 1);

	//Services
	wheelchair_auto_client_ = nh.serviceClient<std_srvs::Empty>(
			"/wheelchair/bb_robot/autonomous_enable");
	wheelchair_manual_client_ = nh.serviceClient<std_srvs::Empty>(
			"/wheelchair/bb_robot/manual_enable");
	follower_enable_client_ = nh.serviceClient<controller::EnableControl>(
			"/ctrl_node/enable_control");
	//we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
	//they won't get any useful information back about its status, but this is useful for tools
	//like nav_view and rviz
	ros::NodeHandle simple_nh("move_base_simple");
	goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
			boost::bind(&UIMoveBase::goalCB, this, _1));

	//we'll assume the radius of the robot to be consistent with what's specified for the costmaps
	private_nh.param("local_costmap/inscribed_radius", inscribed_radius_,
			0.325);
	private_nh.param("local_costmap/circumscribed_radius",
			circumscribed_radius_, 0.46);
	private_nh.param("clearing_radius", clearing_radius_,
			circumscribed_radius_);
	private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

	private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
	private_nh.param("clearing_roatation_allowed", clearing_roatation_allowed_,
			true);
	private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_,
			true);

	//create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
	global_planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap",
			tf_);
	global_planner_costmap_ros_->pause();

	//initialize the global planner
	try
	{
		//check if a non fully qualified name has potentially been passed in
		if (!bgp_loader_.isClassAvailable(global_planner))
		{
			std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
			for (unsigned int i = 0; i < classes.size(); ++i)
			{
				if (global_planner == bgp_loader_.getName(classes[i]))
				{
					//if we've found a match... we'll get the fully qualified name and break out of the loop
					ROS_WARN(
							"Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
							global_planner.c_str(), classes[i].c_str());
					global_planner = classes[i];
					break;
				}
			}
		}

		global_planner_ = bgp_loader_.createClassInstance(global_planner);
		global_planner_->initialize(bgp_loader_.getName(global_planner),
				global_planner_costmap_ros_);
	} catch (const pluginlib::PluginlibException& ex)
	{
		ROS_FATAL(
				"Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s",
				global_planner.c_str(), ex.what());
		exit(1);
	}

	ROS_INFO("MAP SIZE: %d, %d", global_planner_costmap_ros_->getSizeInCellsX(),
			global_planner_costmap_ros_->getSizeInCellsY());

	controller_costmap_ros_.pause();

	//create a local planner
	tc_ros_.initialize("TrajectoryPlannerROS", &tf_, &controller_costmap_ros_);
	markers_handler_.initWriter(global_frame);

	// Start actively updating costmaps based on sensor data
	global_planner_costmap_ros_->start();
	controller_costmap_ros_.start();

	//advertise a service for getting a plan
	make_plan_srv_ = private_nh.advertiseService("make_plan",
			&UIMoveBase::planService, this);

	//advertise a service for clearing the costmaps
	clear_unknown_srv_ = private_nh.advertiseService("clear_unknown_space",
			&UIMoveBase::clearUnknownService, this);

	//advertise a service for clearing the costmaps
	clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps",
			&UIMoveBase::clearCostmapsService, this);

	//initially clear any unknown space around the robot
	global_planner_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 4,
			circumscribed_radius_ * 4);
	controller_costmap_ros_.clearNonLethalWindow(circumscribed_radius_ * 4,
			circumscribed_radius_ * 4);

	//if we shutdown our costmaps when we're deactivated... we'll do that now
	if (shutdown_costmaps_)	//to stop the costmaps when controller is inactive
	{
		ROS_DEBUG_NAMED("ui_move_base", "Stopping costmaps initially");
		global_planner_costmap_ros_->stop();
		controller_costmap_ros_.stop();
	}

	//load any user specified recovery behaviors, and if that fails load the defaults
	if (!loadRecoveryBehaviors(private_nh))
	{
		loadDefaultRecoveryBehaviors();
	}

	//initially, we'll need to make a plan
	movebase_state_ = PLANNING;
	navigation_behavior_ = AUTONOMOUS;
	nav_sub_behavior_ = BRAKE;
	wheelchair_mode_ = COMPUTER; //The wheelchair should be initially in autonomous mode

	//we'll start executing recovery behaviors at the beginning of our list
	recovery_index_ = 0;

	//we're all set up now so we can start the action server
	as_->start();

	dsrv_ = new dynamic_reconfigure::Server<ui_move_base::UIMoveBaseConfig>(
			ros::NodeHandle("~"));
	dynamic_reconfigure::Server<ui_move_base::UIMoveBaseConfig>::CallbackType cb =
			boost::bind(&UIMoveBase::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);
}

void UIMoveBase::reconfigureCB(ui_move_base::UIMoveBaseConfig &config,
		uint32_t level)
{
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	//The first time we're called, we just want to make sure we have the
	//original configuration
	if (!setup_)
	{
		last_config_ = config;
		default_config_ = config;
		setup_ = true;
		return;
	}

	if (config.restore_defaults)
	{
		config = default_config_;
		//if someone sets restore defaults on the parameter server, prevent looping
		config.restore_defaults = false;
	}

	if (global_planner_frequency_ != config.planner_frequency)
	{
		global_planner_frequency_ = config.planner_frequency;
		p_freq_change_ = true;
	}

	if (controller_frequency_ != config.controller_frequency)
	{
		controller_frequency_ = config.controller_frequency;
		c_freq_change_ = true;
	}

	global_planner_patience_ = config.planner_patience;
	controller_patience_ = config.controller_patience;
	conservative_reset_dist_ = config.conservative_reset_dist;

	recovery_behavior_enabled_ = config.recovery_behavior_enabled;
	clearing_roatation_allowed_ = config.clearing_rotation_allowed;
	shutdown_costmaps_ = config.shutdown_costmaps;

	oscillation_timeout_ = config.oscillation_timeout;
	oscillation_distance_ = config.oscillation_distance;

	theta_range_ = config.theta_range;
	num_th_samples_ = config.num_th_samples;
	num_x_samples_ = config.num_x_samples;
	user_vel_cost_scale_ = config.user_vel_cost_scale;
	occ_cost_scale_ = config.occ_cost_scale;
	print_multiple_paths_ = config.print_multiple_paths;
	go_time_ = config.go_time;
	num_paths_ = config.num_paths;
	publish_traj_markers_ = config.publish_traj_markers;
	try_user_vel_first_ = config.try_user_vel_first;
	if (config.base_global_planner != last_config_.base_global_planner)
	{
		nav_core::BaseGlobalPlanner* old_global_planner = global_planner_;
		//initialize the global planner
		ROS_INFO("Loading global planner %s",
				config.base_global_planner.c_str());
		try
		{
			//check if a non fully qualified name has potentially been passed in
			if (!bgp_loader_.isClassAvailable(config.base_global_planner))
			{
				std::vector<std::string> classes =
						bgp_loader_.getDeclaredClasses();
				for (unsigned int i = 0; i < classes.size(); ++i)
				{
					if (config.base_global_planner
							== bgp_loader_.getName(classes[i]))
					{
						//if we've found a match... we'll get the fully qualified name and break out of the loop
						ROS_WARN(
								"Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
								config.base_global_planner.c_str(),
								classes[i].c_str());
						config.base_global_planner = classes[i];
						break;
					}
				}
			}

			global_planner_ = bgp_loader_.createClassInstance(
					config.base_global_planner);

			// wait for the current planner to finish planning
			boost::unique_lock<boost::mutex> lock(global_planner_mutex_);

			delete old_global_planner;
			// Clean up before initializing the new planner
			global_planner_plan_->clear();
			latest_plan_->clear();
			controller_plan_->clear();
			resetState();
			global_planner_->initialize(
					bgp_loader_.getName(config.base_global_planner),
					global_planner_costmap_ros_);

			lock.unlock();
		} catch (const pluginlib::PluginlibException& ex)
		{
			ROS_FATAL(
					"Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s",
					config.base_global_planner.c_str(), ex.what());
			global_planner_ = old_global_planner;
			config.base_global_planner = last_config_.base_global_planner;
		}
	}

	//	if (config.base_local_planner != last_config_.base_local_planner)
	//	{
	//		ui_local_planner::TrajectoryPlannerROS * old_local_planner = & tc_ros_;
	//		//create a local planner
	//		try
	//		{
	//			//check if a non fully qualified name has potentially been passed in
	//			ROS_INFO(
	//					"Loading local planner: %s", config.base_local_planner.c_str());
	//			delete old_local_planner;
	//			// Clean up before initializing the new planner
	//			global_planner_plan_->clear();
	//			latest_plan_->clear();
	//			controller_plan_->clear();
	//			resetState();
	//			tc_ros_.initialize("local_planner", &tf_, &controller_costmap_ros_);
	//		} catch (const pluginlib::PluginlibException& ex)
	//		{
	//			ROS_FATAL(
	//					"Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
	//			tc_ros_ = old_local_planner;
	//			config.base_local_planner = last_config_.base_local_planner;
	//		}
	//	}

	last_config_ = config;
}

void UIMoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	if (navigation_behavior_ == AUTONOMOUS)
	{
		ROS_DEBUG_NAMED("ui_move_base",
				"In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");

		move_base_msgs::MoveBaseActionGoal action_goal;
		action_goal.header.stamp = ros::Time::now();
		action_goal.goal.target_pose = *goal;

		action_goal_pub_.publish(action_goal);
	}
}

void UIMoveBase::clearCostmapWindows(double size_x, double size_y)
{
	/**
	 * @brief  Clears obstacles within a window around the robot
	 * @param size_x The x size of the window
	 * @param size_y The y size of the window
	 */
	tf::Stamped<tf::Pose> global_pose;

	//clear the planner's costmap
	global_planner_costmap_ros_->getRobotPose(global_pose);

	std::vector<geometry_msgs::Point> clear_poly;
	double x = global_pose.getOrigin().x();
	double y = global_pose.getOrigin().y();
	geometry_msgs::Point pt;

	pt.x = x - size_x / 2;
	pt.y = y - size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x + size_x / 2;
	pt.y = y - size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x + size_x / 2;
	pt.y = y + size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x - size_x / 2;
	pt.y = y + size_x / 2;
	clear_poly.push_back(pt);

	global_planner_costmap_ros_->setConvexPolygonCost(clear_poly,
			costmap_2d::FREE_SPACE);

	//clear the controller's costmap
	controller_costmap_ros_.getRobotPose(global_pose);

	clear_poly.clear();
	x = global_pose.getOrigin().x();
	y = global_pose.getOrigin().y();

	pt.x = x - size_x / 2;
	pt.y = y - size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x + size_x / 2;
	pt.y = y - size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x + size_x / 2;
	pt.y = y + size_x / 2;
	clear_poly.push_back(pt);

	pt.x = x - size_x / 2;
	pt.y = y + size_x / 2;
	clear_poly.push_back(pt);

	controller_costmap_ros_.setConvexPolygonCost(clear_poly,
			costmap_2d::FREE_SPACE);
}

bool UIMoveBase::clearUnknownService(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &resp)
{
	/**
	 * @brief  A service call that clears unknown space in the robot's immediate area
	 * @param req The service request
	 * @param resp The service response
	 * @return True if the service call succeeds, false otherwise
	 */
	//clear any unknown space around the robot the same as we do on initialization
	global_planner_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 4,
			circumscribed_radius_ * 4);
	controller_costmap_ros_.clearNonLethalWindow(circumscribed_radius_ * 4,
			circumscribed_radius_ * 4);
	return true;
}

bool UIMoveBase::clearCostmapsService(std_srvs::Empty::Request &req,
		std_srvs::Empty::Response &resp)
{
	/**
	 * @brief  A service call that clears the costmaps of obstacles
	 * @param req The service request
	 * @param resp The service response
	 * @return True if the service call succeeds, false otherwise
	 */
	//clear the costmaps
	global_planner_costmap_ros_->resetMapOutsideWindow(0, 0);
	controller_costmap_ros_.resetMapOutsideWindow(0, 0);
	return true;
}

bool UIMoveBase::planService(nav_msgs::GetPlan::Request &req,
		nav_msgs::GetPlan::Response &resp)
{
	/**
	 * @brief  A service call that can be made when the action is inactive that will return a plan
	 * @param  req The goal request
	 * @param  resp The plan request
	 * @return True if planning succeeded, false otherwise
	 */
	if (as_->isActive())
	{
		ROS_ERROR(
				"ui_move_base must be in an inactive state to make a plan for an external user");
		return false;
	}

	//make sure we have a costmap for our planner
	if (global_planner_costmap_ros_ == NULL)
	{
		ROS_ERROR(
				"ui_move_base cannot make a plan for you because it doesn't have a costmap");
		return false;
	}

	tf::Stamped<tf::Pose> global_pose;
	if (!global_planner_costmap_ros_->getRobotPose(global_pose))
	{
		ROS_ERROR(
				"ui_move_base cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}

	geometry_msgs::PoseStamped start;
	//if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
	if (req.start.header.frame_id == "")
		tf::poseStampedTFToMsg(global_pose, start);
	else
		start = req.start;

	//update the copy of the costmap the planner uses
	clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

	//if we have a tolerance on the goal point that is greater
	//than the resolution of the map... compute the full potential function
	double resolution = global_planner_costmap_ros_->getResolution();
	std::vector<geometry_msgs::PoseStamped> global_plan;
	geometry_msgs::PoseStamped p;
	p = req.goal;
	p.pose.position.y = req.goal.pose.position.y - req.tolerance;
	bool found_legal = false;
	while (!found_legal
			&& p.pose.position.y <= req.goal.pose.position.y + req.tolerance)
	{
		p.pose.position.x = req.goal.pose.position.x - req.tolerance;
		while (!found_legal
				&& p.pose.position.x <= req.goal.pose.position.x + req.tolerance)
		{
			if (global_planner_->makePlan(start, p, global_plan))
			{
				if (!global_plan.empty())
				{
					global_plan.push_back(p);
					found_legal = true;
				}
				else
					ROS_DEBUG_NAMED("ui_move_base",
							"Failed to find a  plan to point (%.2f, %.2f)",
							p.pose.position.x, p.pose.position.y);
			}
			p.pose.position.x += resolution * 3.0;
		}
		p.pose.position.y += resolution * 3.0;
	}

	//copy the plan into a message to send out
	resp.plan.poses.resize(global_plan.size());
	for (unsigned int i = 0; i < global_plan.size(); ++i)
	{
		resp.plan.poses[i] = global_plan[i];
	}

	return true;
}

void UIMoveBase::publish_paths(std::vector<nav_msgs::Path>& paths_vector)
{
	nav_msgs::Path temporal_path;

	if (paths_vector.size() <= 0)
	{
		ROS_WARN("The trajectories vector is empty, will not print any marker");
	}
	else
	{
		// Create the vertices for the points and lines from trajectory
		for (uint32_t n = 0; n < paths_vector.size(); n++)
		{
			temporal_path = paths_vector[n];
			paths_pub_.publish(temporal_path);

		}
	}

}

bool UIMoveBase::get_many_paths(user_intentions::LocalGoalArray & goal_array,
		std::vector<std::vector<geometry_msgs::PoseStamped> > & plan_vector)
{
	/**
	 * @brief  It receives a list of goals and returns a list of paths to those goals from the current robot's position
	 * @param  goal_vector The list of goals to be planned
	 * @param  plan_vector The vector of retrieved plans
	 * @return True if planning succeeded, false otherwise
	 */
	if (as_->isActive())
	{
		ROS_ERROR(
				"ui_move_base must be in an inactive state to make a plan for an external user");
		return false;
	}

	//make sure we have a costmap for our planner
	if (global_planner_costmap_ros_ == NULL)
	{
		ROS_ERROR(
				"ui_move_base cannot make a plan for you because it doesn't have a costmap");
		return false;
	}

	tf::Stamped<tf::Pose> global_pose;
	if (!global_planner_costmap_ros_->getRobotPose(global_pose))
	{
		ROS_ERROR(
				"ui_move_base cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}
	//gets the robot current position
	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(global_pose, start);

	//update the copy of the costmap the planner uses
	clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

	//get a goal as PoseStamped from ui LocalGoalArray
	for (int i = 0; i < num_paths_; i++)
	{
		goal1_ = goal_array.goals[i];
		geometry_msgs::PoseStamped goal;
		tf::Quaternion quat;
		std::vector<geometry_msgs::PoseStamped> global_plan;
		quat = tf::createQuaternionFromRPY(0.0, 0.0, goal1_.pose.theta);
		goal.pose.position.x = goal1_.pose.x;
		goal.pose.position.y = goal1_.pose.y;
		goal.pose.orientation.x = quat.getX();
		goal.pose.orientation.y = quat.getY();
		goal.pose.orientation.z = quat.getZ();
		goal.pose.orientation.w = quat.getW();
		goal.header.frame_id = "/map"; //todo: change this;

		//if we have a tolerance on the goal point that is greater
		//than the resolution of the map... compute the full potential function
		double resolution = global_planner_costmap_ros_->getResolution();
		geometry_msgs::PoseStamped p;//This is to compute a valid goal near to the proposed one in the map.
		p = goal;
		double tolerance = 0.1; //todo:change this
		p.pose.position.y = goal.pose.position.y - tolerance;
		bool found_legal = false;
		while (!found_legal
				&& p.pose.position.y <= goal.pose.position.y + tolerance)
		{
			p.pose.position.x = goal.pose.position.x - tolerance;
			while (!found_legal
					&& p.pose.position.x <= goal.pose.position.x + tolerance)
			{
				if (global_planner_->makePlan(start, p, global_plan))
				{
					if (!global_plan.empty())
					{
						global_plan.push_back(p);
						found_legal = true;
					}
					else
						ROS_DEBUG_NAMED("ui_move_base",
								"Failed to find a  plan to point (%.2f, %.2f)",
								p.pose.position.x, p.pose.position.y);
				}
				p.pose.position.x += resolution * 3.0;
			}
			p.pose.position.y += resolution * 3.0;
		}

		//Put this path into the vector
		plan_vector.push_back(global_plan);
	}
	return true;
}

UIMoveBase::~UIMoveBase()
{
	/**
	 * @brief  Destructor - Cleans up
	 */
	recovery_behaviors_.clear();

	delete dsrv_;

	if (as_ != NULL)
		delete as_;

	if (global_planner_ != NULL)
		delete global_planner_;

	if (global_planner_costmap_ros_ != NULL)
		delete global_planner_costmap_ros_;

	global_planner_thread_->interrupt();
	global_planner_thread_->join();

	delete global_planner_plan_;
	delete latest_plan_;
	delete controller_plan_;
}

bool UIMoveBase::makePlan(const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan)
{
	/**
	 * @brief  Make a new global plan
	 * @param  goal: The goal to plan to (in /map coordinates)
	 * @param  plan: Will be filled in with the plan made by the planner
	 * @return  True if planning succeeds, false otherwise
	 */
	//make sure to set the plan to be empty initially
	plan.clear();

	//since this gets called on handle activate
	if (global_planner_costmap_ros_ == NULL)
	{
		ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
		return false;
	}

	//get the starting pose of the robot
	tf::Stamped<tf::Pose> global_pose;
	if (!global_planner_costmap_ros_->getRobotPose(global_pose))
	{
		ROS_WARN(
				"Unable to get starting pose of robot, unable to create global plan");
		return false;
	}

	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(global_pose, start);

	//if the planner fails or returns a zero length plan, planning failed
	if (!global_planner_->makePlan(start, goal, plan) || plan.empty())
	{
		ROS_DEBUG_NAMED("ui_move_base",
				"Failed to find a  plan to point (%.2f, %.2f)",
				goal.pose.position.x, goal.pose.position.y);
		return false;
	}

	return true;
}

void UIMoveBase::publishZeroVelocity()
{
	/**
	 * @brief  Publishes a velocity command of zero to the base
	 */
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.angular.z = 0.0;
	cmd_vel_pub_.publish(cmd_vel);

}

bool UIMoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
{
	//first we need to check if the quaternion (from given goal position) has nan's or infs
	if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)
			|| !std::isfinite(q.w))
	{
		ROS_ERROR(
				"Quaternion has nans or infs... discarding as a navigation goal");
		return false;
	}

	tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

	//next, we need to check if the length of the quaternion is close to zero
	if (tf_q.length2() < 1e-6)
	{
		ROS_ERROR(
				"Quaternion has length close to zero... discarding as navigation goal");
		return false;
	}

	//next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
	tf_q.normalize();

	tf::Vector3 up(0, 0, 1);

	double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

	if (fabs(dot - 1) > 1e-3)
	{
		ROS_ERROR(
				"Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
		return false;
	}

	return true;
}

geometry_msgs::PoseStamped UIMoveBase::goalToGlobalFrame(
		const geometry_msgs::PoseStamped& goal_pose_msg)
{
	std::string global_frame = global_planner_costmap_ros_->getGlobalFrameID();
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(goal_pose_msg, goal_pose);

	//just get the latest available transform... for accuracy they should send
	//goals in the frame of the planner
	goal_pose.stamp_ = ros::Time();

	try
	{
		tf_.transformPose(global_frame, goal_pose, global_pose);
	} catch (tf::TransformException& ex)
	{
		ROS_WARN(
				"Failed to transform the goal pose from %s into the %s frame: %s",
				goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return goal_pose_msg;
	}

	geometry_msgs::PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	return global_pose_msg;

}

void UIMoveBase::planThread()
{
	ROS_DEBUG_NAMED("ui_move_base_plan_thread", "Starting planner thread...");
	ros::NodeHandle n;
	ros::Rate r(global_planner_frequency_);
	boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
	while (n.ok())
	{
		if (p_freq_change_)
		{
			ROS_INFO("Setting planner frequency to %.2f",
					global_planner_frequency_);
			r = ros::Rate(global_planner_frequency_);
			p_freq_change_ = false;
		}

		//check if we should run the planner (the mutex is locked)
		while (!runPlanner_)
		{
			//if we should not be running the planner then suspend this thread
			ROS_DEBUG_NAMED("ui_move_base_plan_thread",
					"Planner thread is suspending");
			global_planner_cond_.wait(lock);
		}
		//time to plan! get a copy of the goal and unlock the mutex
		geometry_msgs::PoseStamped temp_goal = global_planner_goal_;
		lock.unlock();
		ROS_DEBUG_NAMED("ui_move_base_plan_thread", "Planning...");

		//run planner
		global_planner_plan_->clear();
		bool gotPlan = n.ok() && makePlan(temp_goal, *global_planner_plan_);

		if (gotPlan)
		{
			ROS_DEBUG_NAMED("ui_move_base_plan_thread",
					"Got Plan with %zu points!", global_planner_plan_->size());
			//pointer swap the plans under mutex (the controller will pull from latest_plan_)
			std::vector<geometry_msgs::PoseStamped>* temp_plan =
					global_planner_plan_;

			lock.lock();
			global_planner_plan_ = latest_plan_;
			latest_plan_ = temp_plan;
			last_valid_plan_time_ = ros::Time::now();
			new_global_plan_ = true;

			ROS_DEBUG_NAMED("ui_move_base_plan_thread",
					"Generated a plan from the base_global_planner");

			//make sure we only start the controller if we still haven't reached the goal
			if (runPlanner_)
				movebase_state_ = CONTROLLING;
			if (global_planner_frequency_ <= 0)
				runPlanner_ = false;
			lock.unlock();
		}
		//if we didn't get a plan and we are in the planning state (the robot isn't moving)
		else if (movebase_state_ == PLANNING)
		{
			ROS_DEBUG_NAMED("ui_move_base_plan_thread", "No Plan...");
			ros::Time end_attempt_time = last_valid_plan_time_
					+ ros::Duration(global_planner_patience_);

			//check if we've tried to make a plan for over our time limit
			if (ros::Time::now() > end_attempt_time)
			{
				//we'll move into our obstacle clearing mode
				movebase_state_ = CLEARING;
				publishZeroVelocity();
				recovery_trigger_ = PLANNING_R;
			}
		}

		if (!p_freq_change_ && global_planner_frequency_ > 0)
			r.sleep();

		//take the mutex for the next iteration
		lock.lock();
	}
}

void UIMoveBase::executeCb(
		const move_base_msgs::MoveBaseGoalConstPtr& ui_move_base_goal)
{
	/*** Starts the move_base when a goal is received
	 *  This is the main thread, but must of the work is done in the  "executeCycle(goal, global_plan);"
	 */
//	if (movebase_state_ != BACK)
//		navigation_behavior_ = AUTONOMOUS;
	ROS_DEBUG_NAMED("ui_move_base", "IN execute callback");
	if (!isQuaternionValid(ui_move_base_goal->target_pose.pose.orientation))
	{
		as_->setAborted(move_base_msgs::MoveBaseResult(),
				"Aborting on goal because it was sent with an invalid quaternion");
		return;
	}

	geometry_msgs::PoseStamped goal = goalToGlobalFrame(
			ui_move_base_goal->target_pose);

	//we have a goal so start the planner
	boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
	global_planner_goal_ = goal; //It loads the goal that will be computed in the planning thread
	runPlanner_ = true; //It tells to the planning thread that the new goal is available and the path planner should be executed
	global_planner_cond_.notify_one();
	lock.unlock();

	current_goal_pub_.publish(goal);
	std::vector<geometry_msgs::PoseStamped> global_plan;

	ros::Rate r(controller_frequency_);
	if (shutdown_costmaps_)
	{
		ROS_DEBUG_NAMED("ui_move_base",
				"Starting up costmaps that were shut down previously");
		global_planner_costmap_ros_->start();
		controller_costmap_ros_.start();
	}

	//we want to make sure that we reset the last time we had a valid plan and control
	last_valid_control_time_ = ros::Time::now();
	last_valid_plan_time_ = ros::Time::now();
	last_oscillation_reset_time_ = ros::Time::now();

	ros::NodeHandle n;
	while (n.ok())
	{
		if (c_freq_change_)
		{
			ROS_INFO("Setting controller frequency to %.2f",
					controller_frequency_);
			r = ros::Rate(controller_frequency_);
			c_freq_change_ = false;
		}
		//Check for external requests of changing the goal.
		if (as_->isPreemptRequested())
		{
			if (as_->isNewGoalAvailable())
			{
				//if we're active and a new goal is available, we'll accept it, but we won't shut anything down
				move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

				if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
				{
					as_->setAborted(move_base_msgs::MoveBaseResult(),
							"Aborting on goal because it was sent with an invalid quaternion");
					return;
				}

				goal = goalToGlobalFrame(new_goal.target_pose);

				//we'll make sure that we reset our state for the next execution cycle
				recovery_index_ = 0;
				movebase_state_ = PLANNING;

				//we have a new goal so make sure the planner is awake
				lock.lock();
				global_planner_goal_ = goal;
				runPlanner_ = true;
				global_planner_cond_.notify_one();
				lock.unlock();

				//publish the goal point to the visualizer
				ROS_DEBUG_NAMED("ui_move_base",
						"ui_move_base has received a goal of x: %.2f, y: %.2f",
						goal.pose.position.x, goal.pose.position.y);
				current_goal_pub_.publish(goal);

				//make sure to reset our timeouts
				last_valid_control_time_ = ros::Time::now();
				last_valid_plan_time_ = ros::Time::now();
				last_oscillation_reset_time_ = ros::Time::now();
			}
			else
			{
				//if we've been preempted explicitly we need to shut things down
				resetState();

				//notify the ActionServer that we've successfully preempted
				ROS_DEBUG_NAMED("ui_move_base",
						"Move base preempting the current goal");
				as_->setPreempted();

				//we'll actually return from execute after preempting
				return;
			}
		}

		//we also want to check if we've changed global frames because we need to transform our goal pose
		if (goal.header.frame_id
				!= global_planner_costmap_ros_->getGlobalFrameID())
		{
			goal = goalToGlobalFrame(goal);

			//we want to go back to the planning state for the next execution cycle
			recovery_index_ = 0;
			movebase_state_ = PLANNING;

			//we have a new goal so make sure the planner is awake
			lock.lock();
			global_planner_goal_ = goal;
			runPlanner_ = true;
			global_planner_cond_.notify_one();
			lock.unlock();

			//publish the goal point to the visualizer
			ROS_DEBUG_NAMED("ui_move_base",
					"The global frame for ui_move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f",
					goal.header.frame_id.c_str(), goal.pose.position.x,
					goal.pose.position.y);
			current_goal_pub_.publish(goal);

			//make sure to reset our timeouts
			last_valid_control_time_ = ros::Time::now();
			last_valid_plan_time_ = ros::Time::now();
			last_oscillation_reset_time_ = ros::Time::now();
		}

		//for timing that gives real time even in simulation
		ros::WallTime start = ros::WallTime::now();

		//the real work on pursuing a goal is done here
		bool done = executeCycle(goal, global_plan);

		//if we're done, then we'll return from execute
		if (done)
			return;

		//check if execution of the goal has completed in some way

		ros::WallDuration t_diff = ros::WallTime::now() - start;
		ROS_DEBUG_NAMED("ui_move_base", "Full control cycle time: %.9f\n",
				t_diff.toSec());

		r.sleep();
		//make sure to sleep for the remainder of our cycle time
		if (r.cycleTime() > ros::Duration(1 / controller_frequency_)
				&& movebase_state_ == CONTROLLING)
			ROS_WARN(
					"Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
					controller_frequency_, r.cycleTime().toSec());
	}

	//wake up the planner thread so that it can exit cleanly
	lock.lock();
	runPlanner_ = true;
	global_planner_cond_.notify_one();
	lock.unlock();

	//if the node is killed then we'll abort and return
	as_->setAborted(move_base_msgs::MoveBaseResult(),
			"Aborting on the goal because the node has been killed");
	return;

}

double UIMoveBase::distance(const geometry_msgs::PoseStamped& p1,
		const geometry_msgs::PoseStamped& p2)
{
	return sqrt(
			(p1.pose.position.x - p2.pose.position.x)
					* (p1.pose.position.x - p2.pose.position.x)
					+ (p1.pose.position.y - p2.pose.position.y)
							* (p1.pose.position.y - p2.pose.position.y));
}

bool UIMoveBase::executeCycle(geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& global_plan)
{
	/**
	 * @brief  Performs a control cycle
	 * @param goal A reference to the goal to pursue
	 * @param global_plan A reference to the global plan being used
	 * @return True if processing of the goal is done, false otherwise
	 */
	boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
	//we need to be able to publish velocity commands
	geometry_msgs::Twist cmd_vel;

	//update feedback to correspond to our curent position
	tf::Stamped<tf::Pose> global_pose;
	global_planner_costmap_ros_->getRobotPose(global_pose);
	geometry_msgs::PoseStamped current_position;
	tf::poseStampedTFToMsg(global_pose, current_position);

	//push the feedback out
	move_base_msgs::MoveBaseFeedback feedback;
	feedback.base_position = current_position;
	as_->publishFeedback(feedback);

	//check to see if we've moved far enough to reset our oscillation timeout
	if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
	{
		last_oscillation_reset_time_ = ros::Time::now();
		oscillation_pose_ = current_position;

		//if our last recovery was caused by oscillation, we want to reset the recovery index
		if (recovery_trigger_ == OSCILLATION_R)
			recovery_index_ = 0;
	}

	//check that the observation buffers for the costmap are current, we don't want to drive blind
	if (!controller_costmap_ros_.isCurrent())
	{
		ROS_WARN(
				"[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",
				ros::this_node::getName().c_str());
		publishZeroVelocity();
		return false;
	}

	//if we have a new plan then grab it and give it to the controller
	if (new_global_plan_)
	{
		//make sure to set the new plan flag to false
		new_global_plan_ = false;

		ROS_DEBUG_NAMED("ui_move_base", "Got a new plan...swap pointers");

		//do a pointer swap under mutex
		std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

		boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
		controller_plan_ = latest_plan_;
		latest_plan_ = temp_plan;
		lock.unlock();
		ROS_DEBUG_NAMED("ui_move_base", "pointers swapped!");

		if (!tc_ros_.setPlan(*controller_plan_))
		{
			//ABORT and SHUTDOWN COSTMAPS
			ROS_ERROR(
					"Failed to pass global plan to the controller, aborting.");
			resetState();

			//disable the planner thread
			lock.lock();
			runPlanner_ = false;
			lock.unlock();

			as_->setAborted(move_base_msgs::MoveBaseResult(),
					"Failed to pass global plan to the controller.");
			return true;
		}

		//make sure to reset recovery_index_ since we were able to find a valid plan
		if (recovery_trigger_ == PLANNING_R)
			recovery_index_ = 0;
	}

	//the ui_move_base state machine, handles the control logic for navigation
	switch (movebase_state_)
	{
	//if we are in a planning state, then we'll attempt to make a plan
	case PLANNING:
	{
		boost::mutex::scoped_lock lock(global_planner_mutex_);
		runPlanner_ = true;
		global_planner_cond_.notify_one();
	}
		ROS_DEBUG_NAMED("ui_move_base",
				"Waiting for plan, in the planning state.");
		break;

		//if we're controlling, we'll attempt to find valid velocity commands
	case CONTROLLING:
		ROS_DEBUG_NAMED("ui_move_base", "controlling velocity");
		if (navigation_behavior_ == AUTONOMOUS)
		{
			//check to see if we've reached our goal
			if (tc_ros_.isGoalReached())
			{
				ROS_DEBUG_NAMED("ui_move_base", "Goal reached!");
				resetState();

				//disable the planner thread
				boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
				runPlanner_ = false;
				lock.unlock();

				as_->setSucceeded(move_base_msgs::MoveBaseResult(),
						"Goal reached.");
				//If we arrived succesfully to a destination we willput the wheelchair in turning mode
				nav_sub_behavior_ = TURN;
				return true;
			}

			//check for an oscillation condition
			if (oscillation_timeout_ > 0.0 //if oscillation checking is enabled (>0)
					&& last_oscillation_reset_time_
							+ ros::Duration(oscillation_timeout_) //If the time out finished
					< ros::Time::now())
			{
				publishZeroVelocity(); //Then stop the robot
				movebase_state_ = CLEARING; //And put the robot in CLEARING behavior
				recovery_trigger_ = OSCILLATION_R;
			}
			//XXX compute velocity commands
			if (tc_ros_.computeVelocityCommands(cmd_vel))
			{
				ROS_DEBUG_NAMED("ui_move_base",
						"Got a valid command from the local planner.");
				last_valid_control_time_ = ros::Time::now();
				//make sure that we send the velocity command to the base
				ROS_DEBUG_NAMED("ui_move_base",
						"cmd_vel (%.2f, %.2f)", cmd_vel.linear.x,
						cmd_vel.angular.z);
				cmd_vel_pub_.publish(cmd_vel);
				if (recovery_trigger_ == CONTROLLING_R)
					recovery_index_ = 0;
			}
			else
			{
				ROS_DEBUG_NAMED("ui_move_base",
						"The local planner could not find a valid plan.");
				ros::Time end_attempt_time = last_valid_control_time_
						+ ros::Duration(controller_patience_);

				//check if we've tried to find a valid control for longer than our time limit
				if (ros::Time::now() > end_attempt_time)
				{
					//we'll move into our obstacle clearing mode
					publishZeroVelocity();
					movebase_state_ = CLEARING;
					recovery_trigger_ = CONTROLLING_R;
				}
				else
				{
					//otherwise, if we can't find a valid control, we'll go back to planning
					last_valid_plan_time_ = ros::Time::now();
					movebase_state_ = PLANNING;
					publishZeroVelocity();

					//enable the planner thread in case it isn't running on a clock
					boost::unique_lock<boost::mutex> lock(
							global_planner_mutex_);
					runPlanner_ = true;
					global_planner_cond_.notify_one();
					lock.unlock();
				}
			}
		}

		break;

		//we'll try to clear out space with any user-provided recovery behaviors
	case CLEARING:
		ROS_DEBUG_NAMED("ui_move_base", "In clearing/recovery state");
		//we'll invoke whatever recovery behavior we're currently on if they're enabled
		if (recovery_behavior_enabled_
				&& recovery_index_ < recovery_behaviors_.size()) //If there is any available recovery behavior, do it!!
		{
			ROS_DEBUG_NAMED("ui_move_base_recovery",
					"Executing behavior %u of %zu", recovery_index_,
					recovery_behaviors_.size());
			recovery_behaviors_[recovery_index_]->runBehavior();

			//we at least want to give the robot some time to stop oscillating after executing the behavior
			last_oscillation_reset_time_ = ros::Time::now();

			//we'll check if the recovery behavior actually worked
			ROS_DEBUG_NAMED("ui_move_base_recovery",
					"Going back to planning state");
			movebase_state_ = PLANNING;

			//update the index of the next recovery behavior that we'll try
			recovery_index_++;
		}
		else //If there are no available recovery behaviors just say the possible reason of the error
		{
			ROS_DEBUG_NAMED("ui_move_base_recovery",
					"All recovery behaviors have failed, locking the planner and disabling it.");
			//disable the planner thread
			boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
			runPlanner_ = false;
			lock.unlock();

			ROS_DEBUG_NAMED("ui_move_base_recovery",
					"Something should abort after this.");

			if (recovery_trigger_ == CONTROLLING_R)
			{
				ROS_ERROR(
						"Aborting because a valid control could not be found. Even after executing all recovery behaviors");
				as_->setAborted(move_base_msgs::MoveBaseResult(),
						"Failed to find a valid control. Even after executing recovery behaviors.");
			}
			else if (recovery_trigger_ == PLANNING_R)
			{
				ROS_ERROR(
						"Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
				as_->setAborted(move_base_msgs::MoveBaseResult(),
						"Failed to find a valid plan. Even after executing recovery behaviors.");
			}
			else if (recovery_trigger_ == OSCILLATION_R)
			{
				ROS_ERROR(
						"Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
				as_->setAborted(move_base_msgs::MoveBaseResult(),
						"Robot is oscillating. Even after executing recovery behaviors.");
			}
			resetState();
			return true;
		}
		break;

	default:
		ROS_ERROR(
				"This case should never be reached, something is wrong, aborting");
		resetState();
		//disable the planner thread
		boost::unique_lock<boost::mutex> lock(global_planner_mutex_);
		runPlanner_ = false;
		lock.unlock();
		as_->setAborted(move_base_msgs::MoveBaseResult(),
				"Reached a case that should not be hit in ui_move_base. This is a bug, please report it.");
		return true;
	}

	//we aren't done yet
	return false;
}

void UIMoveBase::input_vel_callback_(const geometry_msgs::TwistConstPtr& vel)
{
	/***
	 * Receives the input velocity command (user's desired velocity).
	 */
	ROS_DEBUG_NAMED("ui_move_base", "In ROS input_vel_callback");
	input_vel_ = *vel;
	geometry_msgs::Twist best_cmd;

	ROS_DEBUG_NAMED("ui_move_base",
			"Navigation behaviour:%d, movebase_state:%d.", navigation_behavior_,
			movebase_state_);
	if ((navigation_behavior_ != AUTONOMOUS)
			|| (navigation_behavior_ == AUTONOMOUS
					&& nav_sub_behavior_ != FORWARD))
	{
		if (correct_user_vel_())
		{
			ROS_DEBUG_NAMED("ui_move_base", "Got a valid  corrected command.");
			//last_valid_control_time_ = ros::Time::now();
			//			if (recovery_trigger_ == CONTROLLING_R)
			//				recovery_index_ = 0;
		}
		else
		{
			ROS_DEBUG_NAMED("ui_move_base",
					"The local planner could not correct user input.");

		}
	}

}

void UIMoveBase::target_person_pos_callback_(
		const pal_msgs::PoseWithVelocityConstPtr& msg)
{
	/***
	 * For the "FOLLOWING a person behavior": Receives the position of the target person.
	 */
	target_person_pos_ = *msg;
}

void UIMoveBase::goal_array_callback_(
		const user_intentions::LocalGoalArrayConstPtr& goal_array_msg)
{
	/***
	 * Receives the list of goals with related probabilities computed by the user intentions node
	 */
	goal_array_.header = goal_array_msg->header;
	//gets a copy of the message
	for (unsigned int i = 0; i < goal_array_msg->goals.size(); i++)
	{
		goal_array_.goals.push_back(goal_array_msg->goals[i]);
	}
	if (print_multiple_paths_)
	{
		std::vector<std::vector<geometry_msgs::PoseStamped> > plan_vector;

		if (get_many_paths(goal_array_, plan_vector))		//compute the path
		{

			std::vector<nav_msgs::Path> paths_vector;
			for (int i = 0; i < num_paths_; i++)
			{
				nav_msgs::Path global_path;
				transform_pose_vector_to_path(plan_vector[i], global_path);
				paths_vector.push_back(global_path);
				publish_paths(paths_vector);
				//std::cout << "goal_proba:" << goal1_.proba << std::endl;

			}
		}
	}

}
void UIMoveBase::transform_pose_vector_to_path(
		const std::vector<geometry_msgs::PoseStamped> & vector,
		nav_msgs::Path & path)
{
	path.header.frame_id = "/map"; //todo:change to the proper frame.
	path.header.stamp = ros::Time::now();
	path.poses.resize(vector.size());
	for (unsigned int i = 0; i < vector.size(); i++)
	{
		path.poses[i] = vector[i];
	}
}
void UIMoveBase::input_voice_callback_(const std_msgs::String::ConstPtr& msg)
{
	/***
	 * Receives a vocal command.
	 */
	//ros::Rate r(2);
	input_voice_.data = msg->data;
	geometry_msgs::Twist cmd;
	if (wheelchair_mode_ == COMPUTER)
	{
		if (msg->data == "go" || msg->data == "join") //in Autonomous go will just
		//receive a goal from destination_inference so nothing has to be done
		//in FOLLOW and FACE it has to go forward
		{
			nav_sub_behavior_ = FORWARD;
		}
		else if (msg->data == "back")
		{
			nav_sub_behavior_ = BACK;
		}
		else if ((msg->data == "joystick") || (msg->data == "manual"))
		{
			//Put the wheelchair in manual mode, (drive with the joystick).
			std_srvs::Empty srv;
			if (wheelchair_manual_client_.call(srv))
			{
				wheelchair_mode_ = JOYSTICK;
				ROS_INFO_NAMED("ui_move_base",
						"wheelchair_manual service call succeeded ");
			}
			else
			{
				ROS_ERROR_NAMED("ui_move_base",
						"Failed wheelchair_manual service call");
			}
			nav_sub_behavior_ = BRAKE; //put ui move base in BRAKE mode

		}
		else if (msg->data == "follow")
		{
			navigation_behavior_ = FOLLOW;
			controller::EnableControl follower_srv;
			follower_srv.request.enabled = true;
			ROS_INFO("calling follower enable service");
			follower_enable_client_.call(follower_srv);
		}
		else if (msg->data == "face")
		{
			navigation_behavior_ = FACE;

		}
		else if (msg->data == "brake")
		{
			nav_sub_behavior_ = BRAKE;

		}
		else if (msg->data == "turn")
		{
			ROS_DEBUG_NAMED("ui_move_base", "setting movebase_state to TURN");
			//navigation_behavior_ = FACE;
			nav_sub_behavior_ = TURN;
		}
		else if (msg->data == "autonomous")
		{
			ROS_DEBUG_NAMED("ui_move_base", "setting AUTONOMOUS mode");
			navigation_behavior_ = AUTONOMOUS;
			nav_sub_behavior_ = BRAKE;
		}

		else
		{
			ROS_WARN_NAMED("ui_move_base",
					"ui_move_base: Vocal command not recognized");
		}
	}
	else //If wheelchair is in joystick mode
	{
		if (msg->data == "autonomous")
		{

			//Put wheelchair in autonomous mode, (drive with the move_base velocity commands).
			std_srvs::Empty srv;
			if (wheelchair_auto_client_.call(srv))
			{
				ROS_INFO_NAMED("ui_move_base",
						"wheelchair_manual service call succeeded ");
				navigation_behavior_ = AUTONOMOUS;
				wheelchair_mode_ = COMPUTER;
				nav_sub_behavior_ = BRAKE;
			}
			else
			{
				ROS_ERROR_NAMED("ui_move_base",
						"Failed wheelchair_manual service call");
			}

		}
		else
		{
			ROS_WARN_NAMED("ui_move_base",
					"Not valid command when wheelchair is in manual mode.");
		}
	}

	//r.sleep();

}

bool UIMoveBase::correct_user_vel_()
{
	/***
	 * This function uses dynamic windows method to check if the velocity command given as input is safe and if it is not then it will
	 * look for a valid command that minimizes the dyn_win_cost function of the dynamic window.
	 */
	ROS_DEBUG_NAMED("ui_move_base", "IN correct_user_vel");
	geometry_msgs::Twist cmd;

	if (nav_sub_behavior_ == BRAKE)
	{
		cmd.angular.z = 0;
		cmd.linear.x = 0;
		ROS_DEBUG_NAMED("ui_move_base", "movebase applying (%.2f, %.2f)",
				cmd.linear.x, cmd.angular.z);
		cmd_vel_pub_.publish(cmd);
		return true;
	}
	else
	{
		if (tc_ros_.isInitialized())
		{
			Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();
			trajectories_vect.clear();
//TURNING BEHAVIOUR??
			if (nav_sub_behavior_ == TURN)
				desired_vel[0] = 0; //To guarantee that if we are in turning behavior the linear vel will be 0.
			else
				desired_vel[0] = input_vel_.linear.x;
//FOLLOWING MODE??
//		if (navigation_behavior_ == FOLLOW)
//		{
//			//Compute the relative distance to the target person
//			double distance = sqrt(
//					target_person_pos_.pose.x * target_person_pos_.pose.x
//							+ target_person_pos_.pose.y
//									* target_person_pos_.pose.y);
//
//		}

			desired_vel[1] = input_vel_.linear.y;
			desired_vel[2] = input_vel_.angular.z;
			ROS_DEBUG_NAMED("ui_move_base", "desired vel(%f,%f)",
					input_vel_.linear.x, input_vel_.angular.z);
//	}
			double occ_cost = tc_ros_.getTrajectory(double(desired_vel[0]),
					double(desired_vel[1]), double(desired_vel[2]), true,
					trajectory_);
			double occ_cost_scale = 1.0;
			if (publish_traj_markers_)
				trajectories_vect.push_back(trajectory_);
			if (occ_cost > 0 && try_user_vel_first_)
			{ //first, we'll check the trajectory_ that the user sent in... if its legal... we'll just follow it
				cmd.linear.x = desired_vel[0];
				cmd.angular.z = desired_vel[2];
				if (publish_traj_markers_)
					markers_handler_.write(trajectories_vect);
				ROS_DEBUG_NAMED("ui_move_base", "Applying (%.2f, %.2f)",
						cmd.linear.x, cmd.angular.z);
				cmd_vel_pub_.publish(cmd);
				return true;
			}

			double dth = (theta_range_) / double(num_th_samples_);
			double dx = desired_vel[0] / double(num_x_samples_);
			double start_th = desired_vel[2] - theta_range_ / 2.0;

			Eigen::Vector3f best = Eigen::Vector3f::Zero();
			double best_dist = DBL_MAX;
			bool trajectory_found = false;

//if we don't have a valid trajectory... we'll start checking others in the angular range specified
			trajectories_vect.clear();
			for (int i = 0; i < num_x_samples_; ++i)
			{
				Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
				check_vel[0] = desired_vel[0] - i * dx;
				check_vel[1] = desired_vel[1];
				for (int j = 0; j < num_th_samples_; ++j)
				{
					check_vel[2] = start_th + j * dth;
					occ_cost = tc_ros_.getTrajectory(double(check_vel[0]),
							double(check_vel[1]), double(check_vel[2]), false,
							trajectory_);
					trajectories_vect.push_back(trajectory_);

					if (occ_cost > 0)
					{
						//if we have a legal trajectory, we'll score it based on its distance to our desired velocity
						Eigen::Vector3f diffs = (desired_vel - check_vel);
						double sq_dist = diffs[0] * diffs[0]
								+ diffs[1] * diffs[1] + diffs[2] * diffs[2];
						double ui_cost = occ_cost_scale * occ_cost
								+ user_vel_cost_scale_ * sq_dist;
						//if we have a trajectory that is better than our best one so far, we'll take it
						if (sq_dist < best_dist)
						{
							best = check_vel;
							best_dist = sq_dist;
							trajectory_found = true;

						}
					}

				}
			}
			if (publish_traj_markers_)
				markers_handler_.write(trajectories_vect);
			if (!trajectory_found && navigation_behavior_ == FOLLOW)
			{
				//We check moving slowly to the front.
				double occ_cost = tc_ros_.getTrajectory(0.2, 0.0, 0.0, false,
						trajectory_);
				if (publish_traj_markers_)
					trajectories_vect.push_back(trajectory_);
				if (occ_cost > 0)
				{ //first, we'll check the trajectory_ that the user sent in... if its legal... we'll just follow it
					cmd.linear.x = desired_vel[0];
					cmd.angular.z = desired_vel[2];
					if (publish_traj_markers_)
						markers_handler_.write(trajectories_vect);
					ROS_DEBUG_NAMED("ui_move_base",
							"movebase applying (%.2f, %.2f)", cmd.linear.x,
							cmd.angular.z);
					cmd_vel_pub_.publish(cmd);
					return true;
				}
				return false;
			}

			cmd.linear.x = best[0];
			cmd.linear.y = best[1];
			cmd.angular.z = best[2];
			ROS_DEBUG_NAMED("ui_move_base", "movebase applying (%.2f, %.2f)",
					cmd.linear.x, cmd.angular.z);
			cmd_vel_pub_.publish(cmd);

			return true;
		}
		else
		{
			ROS_ERROR_NAMED("ui_move_base",
					"The local planner is not initialized, no correction can be done to user input vel");
			return false;
		}
	}

}
bool UIMoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
{
	/**
	 * @brief  Load the recovery behaviors for the navigation stack from the parameter server
	 * @param node The ros::NodeHandle to be used for loading parameters
	 * @return True if the recovery behaviors were loaded successfully, false otherwise
	 */
	XmlRpc::XmlRpcValue behavior_list;
	if (node.getParam("recovery_behaviors", behavior_list))
	{
		if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			for (int i = 0; i < behavior_list.size(); ++i)
			{
				if (behavior_list[i].getType()
						== XmlRpc::XmlRpcValue::TypeStruct)
				{
					if (behavior_list[i].hasMember("name")
							&& behavior_list[i].hasMember("type"))
					{
						//check for recovery behaviors with the same name
						for (int j = i + 1; j < behavior_list.size(); j++)
						{
							if (behavior_list[j].getType()
									== XmlRpc::XmlRpcValue::TypeStruct)
							{
								if (behavior_list[j].hasMember("name")
										&& behavior_list[j].hasMember("type"))
								{
									std::string name_i =
											behavior_list[i]["name"];
									std::string name_j =
											behavior_list[j]["name"];
									if (name_i == name_j)
									{
										ROS_ERROR(
												"A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
												name_i.c_str());
										return false;
									}
								}
							}
						}
					}
					else
					{
						ROS_ERROR(
								"Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
						return false;
					}
				}
				else
				{
					ROS_ERROR(
							"Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
							behavior_list[i].getType());
					return false;
				}
			}

			//if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
			for (int i = 0; i < behavior_list.size(); ++i)
			{
				try
				{
					//check if a non fully qualified name has potentially been passed in
					if (!recovery_loader_.isClassAvailable(
							behavior_list[i]["type"]))
					{
						std::vector<std::string> classes =
								recovery_loader_.getDeclaredClasses();
						for (unsigned int i = 0; i < classes.size(); ++i)
						{
							if (behavior_list[i]["type"]
									== recovery_loader_.getName(classes[i]))
							{
								//if we've found a match... we'll get the fully qualified name and break out of the loop
								ROS_WARN(
										"Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
										std::string(behavior_list[i]["type"]).c_str(),
										classes[i].c_str());
								behavior_list[i]["type"] = classes[i];
								break;
							}
						}
					}

					boost::shared_ptr<nav_core::RecoveryBehavior> behavior(
							recovery_loader_.createClassInstance(
									behavior_list[i]["type"]));

					//shouldn't be possible, but it won't hurt to check
					if (behavior.get() == NULL)
					{
						ROS_ERROR(
								"The ClassLoader returned a null pointer without throwing an exception. This should not happen");
						return false;
					}

					//initialize the recovery behavior with its name
					behavior->initialize(behavior_list[i]["name"], &tf_,
							global_planner_costmap_ros_,
							&controller_costmap_ros_);
					recovery_behaviors_.push_back(behavior);
				} catch (pluginlib::PluginlibException& ex)
				{
					ROS_ERROR(
							"Failed to load a plugin. Using default recovery behaviors. Error: %s",
							ex.what());
					return false;
				}
			}
		}
		else
		{
			ROS_ERROR(
					"The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
					behavior_list.getType());
			return false;
		}
	}
	else
	{
//if no recovery_behaviors are specified, we'll just load the defaults
		return false;
	}

//if we've made it here... we've constructed a recovery behavior list successfully
	return true;
}

//we'll load our default recovery behaviors here
void UIMoveBase::loadDefaultRecoveryBehaviors()
{
	/**
	 * @brief  Loads the default recovery behaviors for the navigation stack
	 */
	recovery_behaviors_.clear();
	try
	{
//we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
		ros::NodeHandle n("~");
		n.setParam("conservative_reset/reset_distance",
				conservative_reset_dist_);
		n.setParam("aggressive_reset/reset_distance",
				circumscribed_radius_ * 4);

//first, we'll load a recovery behavior to clear the costmap
		boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(
				recovery_loader_.createClassInstance(
						"clear_costmap_recovery/ClearCostmapRecovery"));
		cons_clear->initialize("conservative_reset", &tf_,
				global_planner_costmap_ros_, &controller_costmap_ros_);
		recovery_behaviors_.push_back(cons_clear);

//next, we'll load a recovery behavior to rotate in place
		boost::shared_ptr<nav_core::RecoveryBehavior> rotate(
				recovery_loader_.createClassInstance(
						"rotate_recovery/RotateRecovery"));
		if (clearing_roatation_allowed_)
		{
			rotate->initialize("rotate_recovery", &tf_,
					global_planner_costmap_ros_, &controller_costmap_ros_);
			recovery_behaviors_.push_back(rotate);
		}

//next, we'll load a recovery behavior that will do an aggressive reset of the costmap
		boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(
				recovery_loader_.createClassInstance(
						"clear_costmap_recovery/ClearCostmapRecovery"));
		ags_clear->initialize("aggressive_reset", &tf_,
				global_planner_costmap_ros_, &controller_costmap_ros_);
		recovery_behaviors_.push_back(ags_clear);

//we'll rotate in-place one more time
		if (clearing_roatation_allowed_)
			recovery_behaviors_.push_back(rotate);
	} catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL(
				"Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s",
				ex.what());
	}

	return;
}
void UIMoveBase::resetState()
{
	/**
	 * @brief  Reset the state of the ui_move_base action and send a zero velocity command to the base
	 */
	movebase_state_ = PLANNING;
	recovery_index_ = 0;
	recovery_trigger_ = PLANNING_R;
	publishZeroVelocity();

//if we shutdown our costmaps when we're deactivated... we'll do that now
	if (shutdown_costmaps_)
	{
		ROS_DEBUG_NAMED("ui_move_base", "Stopping costmaps");
		global_planner_costmap_ros_->stop();
		controller_costmap_ros_.stop();
	}
}

}
;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ui_move_base_node");
	tf::TransformListener tf(ros::Duration(10));

	ui_move_base::UIMoveBase ui_move_base("ui_move_base", tf);

//ros::MultiThreadedSpinner s;
	ros::spin();

	return (0);

}
