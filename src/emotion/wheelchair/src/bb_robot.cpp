#include <csignal>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "ant_platform.h"
#include <std_srvs/Empty.h>

using namespace std;

class BBRobotNode
{
  private:
    double x_scan_offset_;                      ///< Distance x from robot frame of reference for laser scan position
    double y_scan_offset_;                      ///< Distance y from robot frame of reference for laser scan position

    tf::TransformBroadcaster tf_broadcaster_;   ///< tf (reference frame tracker) transformation broadcaster


  public:

    // The way we communicate with the BB Wheelchair
    AntPlatform proxy_;                         ///< Proxy that implements the RPC over LOS communication with wheelchair
    boost::mutex proxy_lock_;                   ///< Lock to make sure we don't call the proxy_ while a call is still waiting for an answer

    // Standard ROS stuff
    ros::NodeHandle node_handle_;             ///< ROS public interface to create subscribers, publishers, etc
    std::string tf_prefix;
    // ROS services
    ros::ServiceServer aut_enable_service_;
    ros::ServiceServer man_enable_service_;
    // What we publish
    ros::Publisher laser_pub_;                  ///< Publisher of LaserScan msg
    ros::Publisher odom_pub_;                   ///< Publisher of Odometry msg

    sensor_msgs::LaserScan scan_msg_;           ///< LaserScan message to be published
    nav_msgs::Odometry odom_msg_;               ///< Odometry message to be published

    // What we subscribe to
    ros::Subscriber cmd_vel_sub_;               ///< Subscriber to Twist (cmd_vel) message

    geometry_msgs::Twist cmdvel;                ///< The Twist (cmd_vel) received.

	// Ros time for watchdog
	ros::Timer watchdog_timer;

    // Our parameters

    double start_angle_;                        ///< Start angle of laser scan
    double end_angle_;                          ///< End angle of laser scan
    double angle_step_;                         ///< Angle resolution
    int    beams_;                              ///< Quantity of beams (do we need this??)
    double range_min_;                          ///< Minimum laser range
    double range_max_;                          ///< Maximum laser range
    bool   autonomus_mode_;                     ///< Do we set the wheelchair to actually execute the cmd_vel commands
	double watchdog_duration;                   ///> whatchdog duration
	double last_linear_vel_;					///> last linear vel command received
    //!Destructor
	~BBRobotNode()
	    {
		 //Before finishing put the wheelchair in joystick mode
		// Lock proxy to avoid interfering with laser scan reading
			  cout << "Wheelchair in manual mode..." << endl;
		      boost::mutex::scoped_lock lock(proxy_lock_);
		      proxy_.modeSetJoystick();
		      proxy_.modeSetJoystick(); // Seems to need to called twice

	    };

	//! Constructor
    /*!
     * @param host IP of BB wheelchair
     * @param port TCP port to open connection
     * @param timeout Time to wait until breaking connection
     */
    BBRobotNode(std::string host, int port, double timeout): proxy_(host,port,timeout)
    {
      // Create proxy connection (in the initailization)

      // Print the info about the IP, port and timeout
      ROS_INFO_STREAM("BlueBotics IP:" << host << " port:" << port << " timeout:" << timeout);
    };

    //! Service to enable autonomous mode
 	bool aut_enable_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
 	{
 		if (autonomus_mode_!=true){
 			autonomus_mode_ = true;
 			 proxy_.modeSetAutonomous();
 			 ROS_INFO("Wheelchair in autonomous mode.\n");
 		}
 		else{
 			ROS_INFO("The wheelchair already was in autonomous mode\n");
 		}
 		return true;
 	};


 	//! Service to enable manual mode
	bool man_enable_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
	{
	    if (autonomus_mode_){
	    	autonomus_mode_ = false;
	    	proxy_.modeSetJoystick();
	    	proxy_.modeSetJoystick(); // Seems to need to called twice
	    	ROS_WARN("Wheelchair in manual mode... will not execute cmd_vel \n");

	    }
	    else{
	    	ROS_INFO("The wheelchair already was in manual mode\n");
	    };
	    return true;


	};

    //! Initialize node. Read parameters, publish and subscribe topics
    void init()
    {

      // Read node parameters
      ros::NodeHandle private_nh_("~");

	  node_handle_.param("tf_prefix", tf_prefix, std::string(""));

	  if(!tf_prefix.empty())
		tf_prefix = "/"+tf_prefix;


      private_nh_.param("xScanOffset", x_scan_offset_, 0.0);
      private_nh_.param("yScanOffset", y_scan_offset_, 0.0);
      private_nh_.param("startAngle", start_angle_, -M_PI_2);
      private_nh_.param("endAngle", end_angle_, M_PI_2);
      private_nh_.param("beams_", beams_, 361);
      private_nh_.param("rangeMin", range_min_, 0.0);
      private_nh_.param("rangeMax", range_max_, 8.1);
      private_nh_.param("autonomousMode", autonomus_mode_, true);
      private_nh_.param("watchdogDuration", watchdog_duration, 1.);

      {
        boost::mutex::scoped_lock lock(proxy_lock_);
        // Login to wheelchair
        proxy_.login("User", "none");

        // Set to autonomous mode??
        if(autonomus_mode_)
        {
          proxy_.modeSetAutonomous();
          ROS_INFO("Wheelchair in autonomous mode.\n");
        }
        else
        {
          proxy_.modeSetJoystick();
          proxy_.modeSetJoystick(); // Seems to need to called twice
          proxy_.odometryGetPose();
          proxy_.odometrySetPose(0,0,0,0,0,0,0,0,0);
          ROS_WARN("Wheelchair in manual mode... will not execute cmd_vel \n");
        }
      }
      aut_enable_service_ = private_nh_.advertiseService("autonomous_enable", &BBRobotNode::aut_enable_cb, this);
      man_enable_service_ = private_nh_.advertiseService("manual_enable", &BBRobotNode::man_enable_cb, this);
      // Advertise our output (tell the world we have an Odometry msg in the odom topic)
      odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 10);

      // Advertise our output (tell the world we have a LaserScan msg in the base_scan topic)
      laser_pub_ = node_handle_.advertise<sensor_msgs::LaserScan>("base_scan", 1);

      // Setup LaserScan message
      scan_msg_.header.frame_id = tf_prefix + "/base_laser";     // Frame of reference of LaserScan data_sick_count
      // Set values of message
      scan_msg_.angle_min = start_angle_;
      scan_msg_.angle_max = end_angle_;
      scan_msg_.time_increment = 0;       // time between measurements [seconds] - should be updated at each pub
      scan_msg_.range_min = range_min_;
      scan_msg_.range_max = range_max_;

      // Get our input. Ask the world for messages in the cmd_vel topic.
      // Each time a message arrive a call to cmdVelCallback will be made
      //if(autonomus_mode_) // Only if we are in autonomous mode
     // {
        cmd_vel_sub_ = node_handle_.subscribe("cmd_vel", 10,&BBRobotNode::cmdVelCallback, this);
        last_linear_vel_ = 0.0;
        // }


    }//init()

    //! Twist (cmd_vel) callback function
    /*!
     * @param cmd_vel_msg Received message from topic
     */
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
    {
    if(autonomus_mode_) // Only if we are in autonomous mode
      {

    	// Lock proxy to avoid interfering with laser scan reading
    	boost::mutex::scoped_lock lock(proxy_lock_);
    	ROS_DEBUG("Sent (vx,vtheta) [%f %f]", cmd_vel_msg->linear.x, cmd_vel_msg->angular.z);
    	if (cmd_vel_msg->linear.x == 0.0) //If stopping
      	{
    		soft_stop(cmd_vel_msg->angular.z);
      	}

		//ROS_DEBUG("Received cmd_vel msg [linear.x linear.y angular.z] [%f %f %f]", cmd_vel_msg->linear.x , cmd_vel_msg->linear.y,cmd_vel_msg->angular.z);
		//       float vel_linear = cmd_vel_msg->linear.x;
		// Send command to wheelchair
		last_linear_vel_ = cmd_vel_msg->linear.x;
		proxy_.motionSetSpeed(cmd_vel_msg->linear.x, cmd_vel_msg->angular.z);
		watchdog_timer.stop();
		if(watchdog_duration > 0)
			watchdog_timer = node_handle_.createTimer(ros::Duration(watchdog_duration), &BBRobotNode::watchdog_callback, this, true);

    }
    }

    //!Soft stop function
 	//! If the wheelchair receives an abrupt stop signal, ommit this command and slow down softly.
 	//! I know it can be dangerous but it's really uncomfortable to stop abruptly the wheelchair.
    void soft_stop(double angular_vel)
    {
    	ros::Rate rate(8);
		double steps =8.0; //! Number of steps to divide the linear vel change
		double linear_vel= 0.0;
		double max_delta_stop = 0.3; //!The maximum speed allowed to go directly to 0, without soft stop.

		if (last_linear_vel_ >= max_delta_stop )//check if last linear was higher than the allowed value
		{
			for (double var = 1.0; var < steps ; ++var)
			{
				linear_vel = last_linear_vel_*(1 - (var/steps));
				proxy_.motionSetSpeed(linear_vel, angular_vel);
				rate.sleep();
			}

		}
    }

    //! Watchdog callback
    /*!
	 * @param e TimerEvent
	 */
    void watchdog_callback(const ros::TimerEvent&)
	{
	  boost::mutex::scoped_lock lock(proxy_lock_);

      ROS_WARN("Watchdog handle, stopping the robot");
      // Send null command to the wheelchair
      soft_stop(0.0);
      proxy_.motionSetSpeed(0, 0);
      ROS_DEBUG("Sent (vx,vtheta) [0 0]");
	}

    //! Initialize ranges and intensities vectors
    /*!
     * @param num_values Size of vector
     */
    void initVectors(uint32_t num_values)
    {
      //Just fill with 0 ranges and intensities (gmapping assume(d)? a constant number o beams in the LaserScan)
      for (size_t i = 0; i < num_values; i++)
      {
        scan_msg_.ranges.push_back(0);
      }
      for (size_t i = 0; i < num_values; i++)
      {
        scan_msg_.intensities.push_back(0);
      }

      //Adjust message angle increment according to number of values (the points received from the wheelchair are not constant)
      scan_msg_.angle_increment = (scan_msg_.angle_max -scan_msg_.angle_min) / (num_values-1);
      angle_step_ = scan_msg_.angle_increment;
    }


    //! Publish our LaserScan msg
    /*!
     * @param values_scan Array with laser range values
     * @param num_values Quantity of values in the array
     * @param start Timestamp of the laser ranger data
     */
    void publishScan(float *values_scan, uint32_t num_values, ros::Time start)
    {
      // Set timestamp
      scan_msg_.header.stamp = start;
      // Fill the array
      for (size_t i = 0; i < num_values; i++)
      {
        scan_msg_.ranges[i] = (float)values_scan[i];
      }
      // And publish!
      laser_pub_.publish(scan_msg_);
    }

    //! Send TF transformation and Odometry message
    /*!
     * @param pose The current pose of the wheelchair
     * @param start Timestamp of when the pose was taken
     */
    void sendTF(float *pose, double *velocities, ros::Time start)
    {

      static bool first_read = true;          // Is this the first time the function is called
      static ros::Time  time_prev;            // Hold the previous time
      static double prev_x, prev_y, prev_rot; // Hold the previous pose

      double curr_x, curr_y, curr_rot, dt;    // Hold the current pose and delta_time
      ros::Time time_curr;                    // Current time

      if ( first_read )
      {
        first_read = false; // Set flag we have entered the function once
        prev_x = 0.0;
        prev_y = 0.0;
        prev_rot = 0.0;
        time_prev = start;
      }
      else
      {
        // From the second time on get the pose and current time

        curr_x =  pose[0];
        curr_y =  pose[1];
        curr_rot =  pose[2];
        time_curr = start;

//         ROS_INFO("x:%f,y:%f,th:%f",curr_x, curr_y, curr_rot);

        // Compute delta time
        dt = (time_curr - time_prev).toSec();  // in sec

        // And velocities
//         double vx, vy, vth;
//         vx = ( curr_x - prev_x )/dt;
//         vy = ( curr_y - prev_y )/dt;
//         vth = ( curr_rot - prev_rot )/dt;
//         ROS_INFO("dt:%f",dt);
//         ROS_INFO("vx:%f,vy:%f,vth:%f",vx, vy, vth);

        // Save current pose, and current time for the next iteration
        prev_x = curr_x;
        prev_y = curr_y;
        prev_rot = curr_rot;
        time_prev = time_curr;

        //since all odometry is 6DOF we'll need a quaternion created from yaw (orientation)
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose[2]);

        //first, we'll publish the transform over tf
        // Need to have the transformation from base_link to odom
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = start;
        odom_trans.header.frame_id = tf_prefix + "/odom";
        odom_trans.child_frame_id = tf_prefix + "/base_link";
        odom_trans.transform.translation.x = pose[0];
        odom_trans.transform.translation.y = pose[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the odom transform
        tf_broadcaster_.sendTransform(odom_trans);

        // Send the transform between base_link and base_laser
        geometry_msgs::Quaternion base_laser_quat = tf::createQuaternionMsgFromYaw(0.0);
        geometry_msgs::TransformStamped base_laser_trans;
        base_laser_trans.header.stamp = start;
        base_laser_trans.header.frame_id = tf_prefix + "/base_link";
        base_laser_trans.child_frame_id = tf_prefix + "/base_laser";
        base_laser_trans.transform.translation.x = x_scan_offset_;
        base_laser_trans.transform.translation.y = y_scan_offset_;
        base_laser_trans.transform.translation.z = 0.0;
        base_laser_trans.transform.rotation = base_laser_quat;
        //send the laser transform
        tf_broadcaster_.sendTransform(base_laser_trans);

        // Now send Odometry message and publish
        odom_msg_.pose.pose.position.x = pose[0];
        odom_msg_.pose.pose.position.y = pose[1];
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation = odom_quat;
        odom_msg_.header.frame_id = tf_prefix + "/odom";
        odom_msg_.header.stamp = start;

        //The Odometry also has the velocities
        odom_msg_.child_frame_id = tf_prefix + "/base_link";
        odom_msg_.twist.twist.linear.x =  velocities[1];
        odom_msg_.twist.twist.linear.y =  0;
        odom_msg_.twist.twist.angular.z = velocities[2];
        odom_pub_.publish(odom_msg_);
      }

    }

    //! Convert from XY to Polar coordinates
    /*!
     * @param x Point x
     * @param y Point y
     * @param dist Polar distance to point
     * @param angle Polar angle to point
     */
    void convertScanXYtoPolar(const float x, const float y, float &dist, float &angle)
    {
//       float temp_x, temp_y;
//
//       temp_x = x - x_scan_offset_;
//       temp_y = y - y_scan_offset_;
//
//       angle = atan2(temp_y, temp_x);
//       dist = hypot(temp_x, temp_y);

        angle = atan2(y, x);
        dist = hypot(x, y);
    }
};


//! Entry function of our node
int main(int argc, char **argv)
{
  // Check arguments
  if (argc < 4)
  {
    ROS_FATAL_STREAM("Usage : " << argv[0] << " hostname port timeout");
    return -1;
  }

  // Parse command line arguments
  std::string host = argv[1];            // IP of wheelchair
  int         port = atoi(argv[2]);      // Port to open socket
  double      timeout = atof(argv[3]);   // Timeout of connection


  try
  {
    // Initialize ROS
    ros::init(argc, argv, "bb_robot");  // "bb_robot" is the name of our node

    // Initialize our node (opening connection to the wheelchair)
    BBRobotNode bb_robot_node(host, port,timeout);
    bb_robot_node.init();

    // Array of values for holding laser data and pose information
    float values[bb_robot_node.beams_];  //Laser data
    float pose[3] = {0};      //Pose information
    double velocities[3] = {0}; //velocity information
//     double x_vel;
//     double th_vel;
//     double time_vel;

    // Get a default 2D scan: Coordinates | SyncPoint;
    bb_robot_node.proxy_.scanGet();

    // Get a the base velocities to generate ROS odom
    bb_robot_node.proxy_.getSpeed();

    int count;
    // Get SICK count to initialize range and intensities arrays
    count = bb_robot_node.proxy_.scan_data_result_.data_sick_count;
    // count "should" be 361, BUT the wheelchair can return other value, because
    // I believe it returns the number of hits, ie. from the 361 beams it returns the X,Y of the
    // beams that actually hit with something.
    // So I will use the parameter beams_ to initialize our vectors

    bb_robot_node.initVectors(bb_robot_node.beams_);

    // Pull Laser Data at 35Hz .. more less (no guarantee in this, depends in connection, latency, etc).
    ros::Rate loop_rate(35);
    while (bb_robot_node.node_handle_.ok())
    {
                        //Get sick scan from wheelchair
      ros::Time start;
      {
        // Lock proxy to don't interfere if a cmd_vel was executing
        boost::mutex::scoped_lock lock(bb_robot_node.proxy_lock_);

        // Scan2D in default configuration
        bb_robot_node.proxy_.scanGet();

        // Get a the base velocities to generate ROS odom
        bb_robot_node.proxy_.getSpeed();

        // What time is it?
        start = ros::Time::now();

        // Get SICK count to initialize range and intensities arrays
        count = bb_robot_node.proxy_.scan_data_result_.data_sick_count;

        if(count>bb_robot_node.beams_)
          count = bb_robot_node.beams_; // make sure nothing weird happens with the qty

        // Sature range values (needed so they can be discarded if the beam was not hit)
        for (int i = 0; i < bb_robot_node.beams_ ; i++)
        {
          values[i] = bb_robot_node.range_max_;
        }

        // Pass to polar coord (unfortunately we don't have acces to raw data in the wheelchair) we need to
        // make a transformation between points (x,y)
        for (int i = 0; i < count ; i++)
        {
          float x = bb_robot_node.proxy_.scan_data_result_.sick_data[2*i];
          float y = bb_robot_node.proxy_.scan_data_result_.sick_data[2*i+1];
          float dist, alpha;
          bb_robot_node.convertScanXYtoPolar(x, y, dist, alpha);
          // Find to which beam number this point correspond
          int idx = lround((alpha - bb_robot_node.start_angle_) / bb_robot_node.angle_step_);
          if ((idx >= 0) && (idx < bb_robot_node.beams_))
            values[idx] = dist; // Set value
        }

        // Ok, info ready, publish our LaserScan msg
        bb_robot_node.publishScan(values, bb_robot_node.beams_, start);

        // We also need to get the odometry, remember?
        pose[0] = bb_robot_node.proxy_.scan_data_result_.sick_pose[0];
        pose[1] = bb_robot_node.proxy_.scan_data_result_.sick_pose[1];
        pose[2] = bb_robot_node.proxy_.scan_data_result_.sick_pose[2];

        velocities[0] = bb_robot_node.proxy_.motion_speed_result_[0]; ///time
        velocities[1] = bb_robot_node.proxy_.motion_speed_result_[1]; ///linear velocity
        velocities[2] = bb_robot_node.proxy_.motion_speed_result_[2]; ///angular velocity

        //ROS_INFO("x_vel:%f,th_vel:%f",velocities[1], velocities[2]);

        // Publish the Odometry info
        bb_robot_node.sendTF(pose, velocities, start);
      }

      // Attend possible callbaks in queue
      ros::spinOnce();
      // OK sleep i we went too fast
      loop_rate.sleep();
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return -1;
  }

  ROS_INFO("ending bb_robot. success.\n");

  return 0;
}

