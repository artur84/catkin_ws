#include <riskrrt_local_planner/user_vel_handler.h>

namespace riskrrt_local_planner
{

UserVelHandler::UserVelHandler()
{
}

UserVelHandler::~UserVelHandler()
{
}

void UserVelHandler::initReader()
{
	user_vel_sub_ = nodeHandle.subscribe("user_vel", 1,
			&UserVelHandler::userVelCb, this);
}

void UserVelHandler::read(geometry_msgs::Twist& user_vel)
{
	user_vel = user_vel_;
}

void UserVelHandler::userVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
	user_vel_.linear.x = msg->linear.x;
	user_vel_.linear.y = msg->linear.y;
	user_vel_.linear.z = msg->linear.z;
	user_vel_.angular.x = msg->angular.x;
	user_vel_.angular.y = msg->angular.y;
	user_vel_.angular.z = msg->angular.z;
}

} /* namespace riskrrt_local_planner */
