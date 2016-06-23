#include <ui_local_planner/target_vel_handler.h>

namespace ui_local_planner
{

TargetVelHandler::TargetVelHandler()
{
}

TargetVelHandler::~TargetVelHandler()
{
}

void TargetVelHandler::initReader()
{
	vel_sub_ = nodeHandle.subscribe("user_vel", 1,
			&TargetVelHandler::VelCb, this);
}

void TargetVelHandler::initWriter()
{
  vel_pub_= nodeHandle.advertise<geometry_msgs::Twist>("user_safe_vel", 1);
}

void TargetVelHandler::read(geometry_msgs::Twist& vel)
{
	vel = input_vel_;
}

void TargetVelHandler::write(geometry_msgs::Twist& vel)
{
	output_vel_ = vel;
	vel_pub_.publish(output_vel_);
}
void TargetVelHandler::VelCb(const geometry_msgs::Twist::ConstPtr& vel)
{
	input_vel_.linear.x = vel->linear.x;
	input_vel_.linear.y = vel->linear.y;
	input_vel_.linear.z = vel->linear.z;
	input_vel_.angular.x = vel->angular.x;
	input_vel_.angular.y = vel->angular.y;
	input_vel_.angular.z = vel->angular.z;
}

} /* namespace ui_local_planner */
