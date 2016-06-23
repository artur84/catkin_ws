#include <ui_local_planner/user_vel_handler.h>

namespace ui_local_planner
{

UserVelHandler::UserVelHandler()
{
}

UserVelHandler::~UserVelHandler()
{
}

void UserVelHandler::initReader(const std::string &topic_name )
{
	input_sub_ = nodeHandle.subscribe(topic_name, 1,
			&UserVelHandler::InputCb, this);
}

void UserVelHandler::read(geometry_msgs::Twist& user_vel)
{
	user_vel = input_;
}

void UserVelHandler::setNewInputFlag(bool value)
{
	new_input_flag_ = value;
}

bool UserVelHandler::newInputAvailabe()
{
	return new_input_flag_;
}

void UserVelHandler::InputCb(const geometry_msgs::Twist::ConstPtr& msg)
{
	new_input_flag_ = true;
	input_.linear.x = msg->linear.x;
	input_.linear.y = msg->linear.y;
	input_.linear.z = msg->linear.z;
	input_.angular.x = msg->angular.x;
	input_.angular.y = msg->angular.y;
	input_.angular.z = msg->angular.z;
}

} /* namespace ui_local_planner */
