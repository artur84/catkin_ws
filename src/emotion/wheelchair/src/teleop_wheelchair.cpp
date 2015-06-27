#include <ros/ros.h>
#include <turtlesim/Velocity.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class TeleopWheelchair
{
public:
  TeleopWheelchair();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
//   geometry_msgs::Twist twist_msg;
//   twist_msg.linear.x = 0.0;
//   twist_msg.linear.y = 0.;
//   twist_msg.linear.z = 0.;
//   twist_msg.angular.x = 0.;
//   twist_msg.angular.y = 0.;
//   twist_msg.angular.z = 0.0;
  
};


TeleopWheelchair::TeleopWheelchair():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


//   vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopWheelchair::joyCallback, this);

}

void TeleopWheelchair::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//   turtlesim::Velocity vel;
  geometry_msgs::Twist twist_msg;
//   vel.angular = a_scale_*joy->axes[angular_];
//   vel.linear = l_scale_*joy->axes[linear_];
  twist_msg.angular.z = a_scale_*joy->axes[angular_];
  twist_msg.linear.x = l_scale_*joy->axes[linear_];
//   vel_pub_.publish(vel);
  vel_pub_.publish(twist_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopWheelchair teleop_turtle;

  ros::spin();
}