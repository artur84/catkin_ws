#ifndef USER_VEL_HANDLER_H_
#define USER_VEL_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace riskrrt_local_planner
{

class UserVelHandler
{
public:

	UserVelHandler();
	~UserVelHandler();
	virtual void initReader();
	virtual void read(geometry_msgs::Twist& user_vel);

private:
	// we listen on velocity on the user_vel topic
	ros::NodeHandle nodeHandle;
	ros::Subscriber user_vel_sub_;
	geometry_msgs::Twist user_vel_;
	/**
	 * @brief  Callback for receiving velocity data
	 * @param msg A velocity message
	 */
	void userVelCb(const geometry_msgs::Twist::ConstPtr& msg);
};

} /* namespace base_local_planner */
#endif /* USER_VEL_HANDLER_H_ */
