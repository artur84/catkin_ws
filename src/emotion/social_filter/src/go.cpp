#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "go");

  ros::NodeHandle n;

  geometry_msgs::Twist go1, go2, go3, go4, go5, go6, go7, go8, go9; 

  ros::Publisher go_pub1 = n.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
  ros::Publisher go_pub2 = n.advertise<geometry_msgs::Twist>("/robot_2/cmd_vel", 1);
  ros::Publisher go_pub3 = n.advertise<geometry_msgs::Twist>("/robot_3/cmd_vel", 1);
  ros::Publisher go_pub4 = n.advertise<geometry_msgs::Twist>("/robot_4/cmd_vel", 1);
  ros::Publisher go_pub5 = n.advertise<geometry_msgs::Twist>("/robot_5/cmd_vel", 1);
  ros::Publisher go_pub6 = n.advertise<geometry_msgs::Twist>("/robot_6/cmd_vel", 1);
  ros::Publisher go_pub7 = n.advertise<geometry_msgs::Twist>("/robot_7/cmd_vel", 1);
  ros::Publisher go_pub8 = n.advertise<geometry_msgs::Twist>("/robot_8/cmd_vel", 1);
  ros::Publisher go_pub9 = n.advertise<geometry_msgs::Twist>("/robot_9/cmd_vel", 1);

  ros::Rate loop_rate(20);
  ros::Duration(0.5).sleep();
  ros::Time begin;
  begin = ros::Time::now();
  ros::Time present;
  ros::Duration dur1(50.0);
  ros::Duration dur2(51.0);
  bool straight=true;
  while (ros::ok())
  {
    present = ros::Time::now();
    if(present-begin <= dur1){
      straight=true;
    }
    else if(present-begin <= dur2){
      straight=false;
    }
    else begin=ros::Time::now();

    //cout<<begin<<endl;
    //cout<<present<<endl;

    if(straight){
      go1.linear.x = 0.25; go1.angular.z = 0.0;
      go2.linear.x = 0.25; go2.angular.z = 0.0;
      go3.linear.x = 0.25; go3.angular.z = 0.0;
      go4.linear.x = 0.25; go4.angular.z = 0.0;
      go5.linear.x = 0.25; go5.angular.z = 0.0;
      go6.linear.x = 0.25; go6.angular.z = 0.0;
      go7.linear.x = 0.25; go7.angular.z = 0.0;
      go8.linear.x = 0.25; go8.angular.z = 0.0;
      go9.linear.x = 0.25; go9.angular.z = 0.0;
    }
    else{
      go1.linear.x = 0.0; go1.angular.z = 3.14159;
      go2.linear.x = 0.0; go2.angular.z = 3.14159;
      go3.linear.x = 0.0; go3.angular.z = 3.14159;
      go4.linear.x = 0.0; go4.angular.z = 3.14159;
      go5.linear.x = 0.0; go5.angular.z = 3.14159;
      go6.linear.x = 0.0; go6.angular.z = 3.14159;
      go7.linear.x = 0.0; go7.angular.z = 3.14159;
      go8.linear.x = 0.0; go8.angular.z = 3.14159;
      go9.linear.x = 0.0; go9.angular.z = 3.14159;
   }

    go_pub1.publish(go1);
    go_pub2.publish(go2);
    go_pub3.publish(go3);
    go_pub4.publish(go4);
    go_pub5.publish(go5);
    go_pub6.publish(go6);
    go_pub7.publish(go7);
    go_pub8.publish(go8);
    go_pub9.publish(go9);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
