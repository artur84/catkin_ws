#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include "std_msgs/Float64.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"
#include "social_filter/humanSocialSpace.h"
#include "social_filter/humanSocialSpaces.h"
#include "social_filter/int_list.h"
#include "tf/transform_listener.h"

/**
 * This node permits to manage social spaces
 */

using namespace std;

social_filter::humanPoses human_poses_list;
social_filter::humanSocialSpaces human_socialSpaces_list;
social_filter::int_list interactions_list; //The list of interactions (two or more persons in the scenario)

ros::Time no_interaction_time;
ros::Duration no_interaction_duration(0);
ros::Time past_pspace_time;
ros::Duration past_pspace_duration(0);

std_msgs::Float64 interaction_msg;

//Messages retrieved
int path_size=0;
double distRobotToGoal=99;
int status_goal=1;

double pspace_sigma = 0.42; //This is the standard deviation of pspace gausian.

std::vector<double>  past_pspace;
double pspace_mean=0.45;
double past_pspace_mean=0.45;
double pspace_sum;
double last_pspace;
double interaction_var = 0;



/*----- Callback -----*/

void peopleCallback(const social_filter::humanPoses pos)
{
  human_poses_list.humans.clear();
  for (uint i = 0; i < pos.humans.size(); i++){
    human_poses_list.humans.push_back(pos.humans[i]);
  }
}

void intCallback(const social_filter::int_list i_list)
{
    interactions_list.header.frame_id = i_list.header.frame_id;
    interactions_list.header.stamp = i_list.header.stamp;
    interactions_list.formation.clear();
    for (uint i = 0; i < i_list.formation.size(); i++)
    {
	    interactions_list.formation.push_back(i_list.formation[i]);
    }
    //to affect the shape of the gaussian depending on the value of the task
    // for(uint k=0;k<interactions_list.formation.size();k++){
    //   if(interactions_list.formation[k].type==VISVIS || interactions_list.formation[k].type==HOI){
    //     //if we dont want effect we put the same angle + pi/2
    //     interactions_list.formation[k].sd_y*=task_factor( interactions_list.formation[k].angle, interactions_list.formation[k].angle + 1.57 );
    //   }
    // }
}

void distToGoalCallback(const std_msgs::Float64 msg)
{
    distRobotToGoal = msg.data;
}

void pathSizeCallBack(const std_msgs::Int32 msg)
{
    path_size= msg.data;
}

void statusGoalCallBack(const std_msgs::Int32 msg)
{
    status_goal= msg.data;
}

/*----- Other functions -----*/

double calculateSocialSpaceSize_beta(){

  pspace_sigma = past_pspace_mean;

  //Change size of personal space every 5 seconds
  past_pspace_duration = ros::Time::now() - past_pspace_time;
  if(past_pspace_duration.sec > 5.0){
	  past_pspace_mean = pspace_mean;
	  past_pspace_time = ros::Time::now();
  }

  return pspace_sigma;
}

//Former function to calculate social spaces
double calculateSocialSpaceSize(){

  last_pspace = (path_size/25.0)*0.45;//increase speed at which pspace is reduced : (pathsize/max_pathsize)

  last_pspace = std::max(last_pspace, 0.1);

  if(path_size>25.0) last_pspace=0.45;

  //if(last_pspace<0) last_pspace=0;
  pspace_sigma = past_pspace_mean;

  if(distRobotToGoal > 3.0){//eviter de voir se retrecir les pspace quand le robot bloque a qq metres du goal
      if(past_pspace.size() > 5){
	      for (int k=0;k<past_pspace.size()-1;k++){
		      past_pspace[k] = past_pspace[k+1];
	      }
	      past_pspace.pop_back();
      }
      past_pspace.push_back(last_pspace);
  }

  pspace_sum = 0.0;
  for(int l=0;l<past_pspace.size();l++){
	  pspace_sum += past_pspace[l];
  }
  pspace_mean = pspace_sum/past_pspace.size();

  past_pspace_duration = ros::Time::now() - past_pspace_time;
  if(pspace_mean <= past_pspace_mean || (/*pspace_mean > past_pspace_mean &&*/
      past_pspace_duration.sec > 5.0)){
	  past_pspace_mean = pspace_mean;
	  past_pspace_time = ros::Time::now();
  }

  /*
  if(path.size()<=1){
	  ROS_INFO("No valid path");
	  first_zero_in_path = ros::Time::now();
	  //ROS_INFO("Reinit without Ps or Is");
	  //ignorePs = 0;
	  //if(rrt.pspace_attribute > 0.2)
	    //rrt.pspace_attribute -= 0.05;
	  //rrt.reInit(robot_origin, goal, ppV, grid, timeLoc, ignorePs);
  }
  */

  return pspace_sigma;

}

void calculateInteractionSize(){
  /*
    duration_since_zero = ros::Time::now() - first_zero_in_path;
    if(path.size()>10){
	    duration_since_zero = first_zero_in_path - first_zero_in_path;//set to 0
    }
    if(duration_since_zero.sec > 10.0){
	    ignorePs = 1;
    }
    */

    no_interaction_duration = ros::Time::now() - no_interaction_time;
    //start_duration = ros::Time::now() - start_time;
    if(no_interaction_duration.sec > 5.0 /*|| start_duration.sec < 5.0*/)
	interaction_var = 0.5 * pspace_sigma + 0.4;
    /*
    else
	    interaction_var = 0;
    */

    if(pspace_sigma < 0.15){
	interaction_var = 0;
	no_interaction_time = ros::Time::now();
    }

    /*
    else if(path.size()==1){
	    ROS_INFO("1 nodes path = bug");
	    ROS_INFO("Reinit");
	    rrt.reInit(robot_origin, goal, ppV, grid, timeLoc, ignorePs);
    }
    */

}

void init(){
  //trick to reinit pspace marker
  for(int z=0;z<past_pspace.size();z++)
    past_pspace[z]=0.6;
  ////

  pspace_sigma= 0.6;
  interaction_var = 0.6;

}

/*----- Main -----*/

int main(int argc, char **argv)
{
  int nbPersons;
  std::vector<social_filter::humanSocialSpace> human_space_measure_list;

  ros::init(argc, argv, "socialSpaceManage");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle n;

  human_space_measure_list.clear();
  private_nh.param("nbPersons", nbPersons,0); cout << "space manage sh " <<nbPersons<<endl;
  for(int i=1;i<=nbPersons;i++){
    social_filter::humanSocialSpace new_hss;
    std::ostringstream flux_i;
    flux_i<< i;
    std::string str_i = flux_i.str();

    double size,sigma_h,sigma_r,sigma_s;
    private_nh.param("human_"+str_i+"_size", size,0.0);
    private_nh.param("human_"+str_i+"_sigma_h", sigma_h,1.0);
    private_nh.param("human_"+str_i+"_sigma_r", sigma_r,0.5);
    private_nh.param("human_"+str_i+"_sigma_s", sigma_s,2.0/3.0);
    private_nh.param("human_"+str_i+"_attractiveness", new_hss.attractiveness,5);

    new_hss.size=(float) size;
    new_hss.sigma_h=(float) sigma_h;
    new_hss.sigma_r=(float) sigma_r;
    new_hss.sigma_s=(float) sigma_s;

    human_space_measure_list.push_back(new_hss);
  }

  no_interaction_time=ros::Time::now();
  past_pspace_time =ros::Time::now();

  //Human position (could be useful)
  ros::Subscriber sub = n.subscribe("human_poses", 1, peopleCallback);

  //Interactions list
  ros::Subscriber sub_int = n.subscribe("interaction_list", 1, intCallback);

  //Figure get from RRT
  ros::Subscriber sub_path = n.subscribe("rosplanner_static/path_size", 1, pathSizeCallBack);
  ros::Subscriber sub_dist = n.subscribe("rosplanner_static/dist_robot_to_goal", 1, distToGoalCallback);
  ros::Subscriber sub_sttg = n.subscribe("rosplanner_static/status_goal", 1, statusGoalCallBack);

  ros::Publisher sspace_pub = n.advertise<social_filter::humanSocialSpaces>("socialSpaces_list", 1);

  ros::Publisher interaction_pub = n.advertise<std_msgs::Float64>("interaction_msg", 1);

  ros::Rate r(3);

  std::string options;

  while (ros::ok())
  {
    //Create social spaces for each human
    human_socialSpaces_list.socialSpaces.clear();
    for(int i = 0;i < human_poses_list.humans.size() ;i++){
      social_filter::humanSocialSpace human_socialSpace;

      human_socialSpace.header.frame_id= human_poses_list.humans[i].header.frame_id;
      human_socialSpace.header.stamp= human_poses_list.humans[i].header.stamp;
      human_socialSpace.id=  human_poses_list.humans[i].id;
      human_socialSpace.human_id= human_poses_list.humans[i].id;

      //Verify that is correct
      if(status_goal==1){
        init();
        human_socialSpace.size=pspace_sigma;
      }else{
        human_socialSpace.size=calculateSocialSpaceSize();
      }

      if(human_space_measure_list.size()>i){
        human_socialSpace.sigma_h=human_space_measure_list[i].sigma_h;
        human_socialSpace.sigma_r=human_space_measure_list[i].sigma_r;
        human_socialSpace.sigma_s=human_space_measure_list[i].sigma_s;
        human_socialSpace.attractiveness=human_space_measure_list[i].attractiveness;
      }else{
        human_socialSpace.sigma_h=1.0;
        human_socialSpace.sigma_r=0.3;
        human_socialSpace.sigma_s=0.6;
        human_socialSpace.attractiveness=5;
      }
// 	cout<<"Status goal "<<status_goal<<endl;
// 	cout<<"Social space psSigma "<<pspace_sigma<<endl;
// 	cout<<"Social space path_size "<<path_size<<endl;
// 	cout<<"Social space distRobotToGoal "<<distRobotToGoal<<endl;

      human_socialSpaces_list.socialSpaces.push_back(human_socialSpace);
    }

    //Publish social spaces of each human
    sspace_pub.publish(human_socialSpaces_list);

    calculateInteractionSize();
    interaction_msg.data = interaction_var;//testing, interaction is supposed to have its own value
    interaction_pub.publish(interaction_msg);

    ros::spinOnce();
    r.sleep();

  }

  return 0;
}

