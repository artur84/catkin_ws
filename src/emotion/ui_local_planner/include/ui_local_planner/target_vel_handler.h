#ifndef TARGET_VEL_HANDLER_H_
#define TARGET_VEL_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace ui_local_planner
{

class TargetVelHandler
{
public:

	TargetVelHandler();
	~TargetVelHandler();

	/**
	 * @brief  Init the subscriber to read the input from user
	 */
	virtual void initReader();
	/**
	 * @brief  Init the publisher to write the safe velocity command
	 */
	virtual void initWriter();

	/**
	 * @brief  reads the input from the user
	 * @param user_vel, Twist() reference to the variable where we want store the input velocity
	 */
	virtual void read(geometry_msgs::Twist& vel);

	/**
	* @brief  writes the safe velocity value
	* @param msg, Twist() the "safe" velocity tu be published
	*/
	virtual void write(geometry_msgs::Twist& vel);



private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber vel_sub_; 	//The ROS subscriber to the velocity command given by the user
	ros::Publisher vel_pub_; 	//The ROS publisher of the velocity command checked by the collision checker
	geometry_msgs::Twist input_vel_;	//INPUT velocity command given by the user
	geometry_msgs::Twist output_vel_;	//OUTPUT velocity command checked by the collision checker
	/**
	 * @brief  Callback for receiving velocity data
	 * @param msg, Twist(), The velocity input comming from user
	 */
	void VelCb(const geometry_msgs::Twist::ConstPtr& msg);


};

} /* namespace base_local_planner */
#endif /* TARGET_VEL_HANDLER_H_ */
