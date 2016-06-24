#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"
#include "social_filter/int_data.h"
#include "social_filter/int_list.h"
#include "social_fcns.h"

#include <std_msgs/Float64.h>

int object_flag= 0; //this is to take into account interactions with objects
/*
 * This node detects interactions based in humans positions and orientations
 * It subscribes to humanPoses and publishes int_list
 */

social_filter::humanPoses myPoses;
geometry_msgs::PoseWithCovarianceStamped robot_state;
bool robot_state_ready=false;
double interaction_space;

//Function to take into account the angle of the robot in the f-formation shape 
double task_factor(double angle_interaction, double angle_robot){
  
  return 1- 0.7*fabs(cos(angle_interaction-angle_robot));
}


void peopleCallback(const social_filter::humanPoses pos)
{

  myPoses.humans.clear();
  for(unsigned int i=0;i<pos.humans.size();i++)
    {
      myPoses.humans.push_back(pos.humans[i]);
    }
}

void robotCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{

  robot_state = msg; 
  robot_state_ready= true;
}

void interactionCallback(const std_msgs::Float64 msg)
{

  interaction_space = msg.data; 

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "formations");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("human_poses", 100, peopleCallback);
 
  //We will hear robot state
  //To do a quick version we are using amcl_pose which is in map frame
  //maybe the best solution must to take into account the odom topic of robot 
  ros::Subscriber sub_robot = n.subscribe("amcl_pose",100,robotCallback);
  
  ros::Subscriber interaction_sub = n.subscribe("/robot_0/rosplanner_static/interaction_msg", 1, interactionCallback);
  
  ros::Publisher interaction_pub = n.advertise<social_filter::int_list>("interaction_list", 1);

  ros::Rate r(3); //maybe we can wait more time to detect interactions

  tf::TransformListener listener; //listener must be alive all time

  int form_matrix[MAX_P][MAX_P]; //To register detected f-formations (1 detected - non detected)
   
 while (ros::ok())
  {
    //initializing with zero matrix
    for(uint i=0; i<MAX_P; i++)
      for(uint j=0; j<MAX_P; j++)
	form_matrix[i][j]=0;
    
    social_filter::int_list my_list;
    std::vector<social_filter::int_data> int_obj;//to keep interactions with objects

    //----------------------------Detect interactions with humans and objects

    for(uint i=1; i<myPoses.humans.size(); i++){

        for(uint j=i+1; j<=myPoses.humans.size();j++){
	  social_filter::int_data my_interaction;
	  social_filter::humanPose human1;
	  social_filter::humanPose human2;
	
	  //As poses are defined in local frames we need to change to global frame
	  transformPosetoMap(&listener, myPoses.humans[i-1], &human1);
	  transformPosetoMap(&listener, myPoses.humans[j-1], &human2);

	  //TODO: Update also information about humans' speed
	 
	  my_interaction.type = EvFormation(human1.x,human1.y,human1.theta,0.0,0.0,
					  human2.x,human2.y,human2.theta,0.0,0.0,
					  &my_interaction, interaction_space);
	  //std::cout<<"human formation: human 1"<<human1.x<<" "<<human1.y<<" "<<human1.theta<<std::endl;
	  //	  std::cout<<"human formation: human 2"<<human2.x<<" "<<human2.y<<" "<<human2.theta<<std::endl;

	  if(my_interaction.type>0)
	    {
	      my_interaction.id_members.push_back(i-1);
	      my_interaction.id_members.push_back(j-1);
	      //here it is calculated a deformation for the Vis-a-Vis formation depending on the 
	      //robot state, mainly its angle in \map frame
	            //commente pour test //if(robot_state_ready==true && (my_interaction.type==VISVIS || my_interaction.type==HOI)){
		//if we dont want effect we put the same angle + pi/2
		//	std::cout << "value of orientation"<< tf::getYaw(robot_state.pose.pose.orientation)*57.3 << std::endl;
			//commente pour test //my_interaction.sd_y*=task_factor( my_interaction.angle,tf::getYaw( robot_state.pose.pose.orientation));
	         //commente pour test //}
		    // std::cout<<"values of interaction, sd.x: "<<my_interaction.sd_x  <<"sd.y: "<<my_interaction.sd_y  <<std::endl;
	      my_list.formation.push_back(my_interaction);
	      form_matrix[i-1][j-1]=1;
	      form_matrix[j-1][i-1]=1;
	     
	      
	    }

	}

	//-----------To detect interactions with objects 
	//TODO: add publisher of objects and reader of objects
	if(object_flag==1){
	  social_filter::int_data interactionAux;
	  social_filter::humanPose humanAux;
	  double obj_x=.2,obj_y=15.5,obj_theta=-1.57;
	  transformPosetoMap(&listener, myPoses.humans[i-1], &humanAux);
	  interactionAux.type = EvObjInt(humanAux.x,humanAux.y,humanAux.theta,0.0,0.0,
				       obj_x,obj_y,obj_theta, &interactionAux);
	  if(interactionAux.type==HOI){
	    interactionAux.id_members.push_back(0);
	    interactionAux.id_members.push_back(i-1);
	    int_obj.push_back(interactionAux);
	  }
	}
	//-------interactions with objects

	
    }//----------endfor i<myPoses.humans.size()
   
    //Post-Processing of f-formations to detect groups > 2
    //only if there are at least two formations
    if( my_list.formation.size()>1){
      bool cluster=1;
      unsigned int p1=0;
      unsigned int p2=1;
      while( cluster!=0){
	cluster=0;
	p1=0;
	while (p1 < my_list.formation.size()-1){
	  p2=p1 + 1;
	  while(p2 < my_list.formation.size()){
	    if(group_tightness(my_list.formation[p1],my_list.formation[p2],form_matrix) ){
	      social_filter::int_data aux;
	      elements(my_list.formation[p1],my_list.formation[p2],&aux);
	      my_list.formation[p1]=aux;
	      my_list.formation.erase(my_list.formation.begin() + p2);
	      cluster=1;
	    }
	    else p2++;
	  }
	  p1++;
	}
      }//------------end while  cluster!=0
    }

  
    //--------Checking if we have a circular formation and creating 
    //--------correct values for the gaussian
    for(uint m=0; m < my_list.formation.size();m++){
      if(my_list.formation[m].type==CIRCULAR)
	{
	  //here we must be careful with the id's because they
	  //could be different of the position in the list.
	  //Calculate centroid of human positions
	  double c_x=0.0;
	  double c_y=0.0;
	  std::vector<double> vector_x;
	  std::vector<double> vector_y;
	  for(uint n=0;n < my_list.formation[m].id_members.size();n++ ){
	    int index=my_list.formation[m].id_members[n];	    
	    social_filter::humanPose h1;
	    transformPosetoMap(&listener, myPoses.humans[index], &h1);
	    c_x+=h1.x;
	    c_y+=h1.y;	    
	    vector_x.push_back(h1.x);
	    vector_y.push_back(h1.y);	    
	  }
	 
	  //Computing Gaussian parameters
	  my_list.formation[m].media_x=c_x/ my_list.formation[m].id_members.size();
	  my_list.formation[m].media_y=c_y/ my_list.formation[m].id_members.size();

	  //computing minimum distance from the center
	  double d_min=0.0,d=0.0;
	  d_min=sqrt(pow( my_list.formation[m].media_x-vector_x[0],2)+ pow( my_list.formation[m].media_y-vector_y[0],2) );
	  for(uint s=1;s<vector_x.size();s++){
	    d=sqrt(pow( my_list.formation[m].media_x-vector_x[s],2)+ pow( my_list.formation[m].media_y-vector_y[s],2) );
	   
	    if (d < d_min) d_min=d;
	  }
	  /////////////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////////////////////////////////////////////////////////////
	  my_list.formation[m].sd_x=interaction_space*d_min;
	  my_list.formation[m].sd_y=interaction_space*d_min;
	  /////////////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////////////////////////////////////////////////////////////
	
	}
    }//-------- m < my_list.formation.size()
   
    //Adding interaction with objects to list  
    //it must be done after 
    for(uint k=0; k< int_obj.size();k++)
      my_list.formation.push_back(int_obj[k]);

    my_list.header.frame_id="/map";
    my_list.header.stamp=ros::Time::now();
    interaction_pub.publish(my_list);
    ros::spinOnce();
    r.sleep();
  }
  
 return 0;
}
