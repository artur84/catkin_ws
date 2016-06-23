#ifndef USER_VEL_HANDLER_H_
#define USER_VEL_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace ui_local_planner
{

class UserVelHandler
{
public:

	UserVelHandler();
	~UserVelHandler();

	/**
	 * @brief  Init the subscriber to read the input from user
	 */
	virtual void initReader(const std::string &topic_name );

	/**
	 * @brief  reads the input from the user
	 * @param user_vel, Twist() reference to the variable where we want store the input velocity
	 */
	virtual void read(geometry_msgs::Twist& user_vel);

	/**
	* @brief  check if a new input from the user is available.
	* @return new_input_flag_, bool, True if a new velocity input was received since last reading.
	*/
	virtual bool newInputAvailabe();

	virtual void setNewInputFlag(bool value);


protected:
	ros::NodeHandle nodeHandle;
	ros::Subscriber input_sub_; 	//The ROS subscriber to the velocity command given by the user
	geometry_msgs::Twist input_;	//INPUT velocity command given by the user (we will use just the angular part)
	bool new_input_flag_; //If the velocity value asked by the user has changed since last reading or not.

	/**
	 * @brief  Callback for receiving velocity data
	 * @param msg, Twist(), The velocity input comming from user
	 */
	void InputCb(const geometry_msgs::Twist::ConstPtr& msg);


};

} /* namespace base_local_planner */
#endif /* USER_VEL_HANDLER_H_ */
