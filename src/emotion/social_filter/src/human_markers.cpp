#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"
#include "social_filter/int_list.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include "social_fcns.h"

/**
 * This node permits to visualize humans and interactions in RVIZ
 */
social_filter::humanPoses myPoses;
social_filter::int_list myList;

double cell_size= .1; //in meters
int int_marker_w=60;
int int_marker_h=60;
int ps_marker_w=100;
int ps_marker_h=50;

double pspace_msg;

//this function it is here to take into account the angle of the robot in 
double task_factor(double angle_interaction, double angle_robot){
  
  return 1- 0.7*fabs(cos(angle_interaction-angle_robot));
}

void text_marker(int type,int id,double x, double y, visualization_msgs::MarkerArray *mc )
{
  visualization_msgs::Marker marker;
   // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 2.0;

  marker.lifetime = ros::Duration(0.5);
  marker.id = id;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the frame ID and timestamp.  
  marker.header.frame_id =  "/map";
  marker.header.stamp =  ros::Time::now();

  marker.ns = "interaction_text";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  switch(type){
  case VISVIS:
    marker.text="Vis-a-Vis";
    break;
  case V_FORM: 
    marker.text="V-shape";
    break;
  case L_FORM :
    marker.text="L-shape";
    break;
  case C_FORM :
    marker.text="C-shape";
    break;
  case CIRCULAR :
    marker.text="Group > 2";
    break;
  case HOI:
    marker.text="Int. with object";
    break;
  default:
    marker.text="Not recognized";
    break;
  }
  //add new marker
  mc->markers.push_back(marker);

}




void ips_markers(visualization_msgs::MarkerArray *mc )
{
int ips_marker_w=80;
int ips_marker_h=80;
  /***
    *Create Markers for each ips 
    ***/
   for(uint i=0;i<myPoses.humans.size();i++)
     {
       visualization_msgs::Marker marker;
       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale.x = 0.12;//1.0;
       marker.scale.y = 0.12;//1.0;
       marker.scale.z = 0.2;//1.0;

       // Set the color -- be sure to set alpha to something non-zero!
       marker.color.r = 1.0f;
       marker.color.g = 0.0f;
       marker.color.b = 0.0f;
       marker.color.a = 0.5;

       marker.lifetime = ros::Duration(0.5);
       marker.id = i;

       // Set the marker action.  Options are ADD and DELETE
       marker.action = visualization_msgs::Marker::ADD;

       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
       marker.pose.position.x = 0.0;
       marker.pose.position.y = 0.0;
       marker.pose.position.z = 0.0;

       // Set the frame ID and timestamp.  
       marker.header.frame_id =  myPoses.humans[i].header.frame_id;//"/map";
       marker.header.stamp = myPoses.humans[i].header.stamp;// ros::Time::now();

       marker.ns = "ips";
       marker.type = visualization_msgs::Marker::POINTS;

       //Set points for the ips

       double min_x= myPoses.humans[i].x -(ips_marker_w* cell_size/2.0);
       double min_y= myPoses.humans[i].y-(ips_marker_h* cell_size/2.0);
	 
       for(int m=0; m<=ips_marker_h; m++)
	 for(int n=0; n<= ips_marker_w; n++)
	   {
	     geometry_msgs::Point p;
	     p.x = min_x +  cell_size*n;
	     p.y =  min_y +  cell_size*m;
	
	     p.z = ips(p.x,p.y,0.0,myPoses.humans[i].x,myPoses.humans[i].y,myPoses.humans[i].theta,2.5);
	     //+ PSpace(p.x,p.y,myposes.humans[i].x,myposes.humans[i].y,myposes.humans[i].theta);
	     if(p.z > 0.5)
	       marker.points.push_back(p);	        	
	   }

       //add new marker
       mc->markers.push_back(marker);

     }

}



void peopleCallback(const social_filter::humanPoses pos)
{
  myPoses.humans.clear();
  for(uint i=0;i<pos.humans.size();i++)
    {
      myPoses.humans.push_back(pos.humans[i]);
    }
}

void intCallback(const social_filter::int_list i_list)
{
  myList.header.frame_id = i_list.header.frame_id;
  myList.header.stamp = i_list.header.stamp;
  myList.formation.clear();
  for(uint i=0;i<i_list.formation.size();i++)
  {
    myList.formation.push_back(i_list.formation[i]);   
  }
 //to affect the shape of the gaussian depending on the value of the task
  // for(uint k=0;k<myList.formation.size();k++){
  //   if(myList.formation[k].type==VISVIS || myList.formation[k].type==HOI){
  //     //if we dont want effect we put the same angle + pi/2
  //     myList.formation[k].sd_y*=task_factor( myList.formation[k].angle, myList.formation[k].angle + 1.57 );
  //   }
  // }
}

void pspaceCallback(const std_msgs::Float64 msg)
{
	pspace_msg = msg.data;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "humanMarkers");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("human_poses", 1, peopleCallback);
  ros::Subscriber sub_int=n.subscribe("interaction_list", 1, intCallback);
  ros::Subscriber pspace_sub = n.subscribe("/robot_0/rosplanner_static/pspace_msg", 1, pspaceCallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("human_markers", 1);
  ros::Rate r(3);

  std::string options;
  if(argc>1)
    options=argv[1];
  else
    options="1000";
    

  while (ros::ok())
    {
      visualization_msgs::MarkerArray ma;

      /***
       *Create Markers for each human ***************************************
       ***/
      if(options[0]=='1'){
	for(uint i=0;i<myPoses.humans.size();i++)
	  {
	    visualization_msgs::Marker marker;
 
	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 1.0f;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration(0.5);
	    marker.id = i;

	    // Set the marker action.  Options are ADD and DELETE
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = myPoses.humans[i].x;
	    marker.pose.position.y = myPoses.humans[i].y;
	    marker.pose.position.z = 0;
	  
	    geometry_msgs::Quaternion local_orientation=  tf::createQuaternionMsgFromYaw(myPoses.humans[i].theta + 1.57);	
	    //TODO: why 1.57?
	    marker.pose.orientation= local_orientation;

	    // Set the frame ID and timestamp.  
	    marker.header.frame_id = myPoses.humans[i].header.frame_id;
	    marker.header.stamp = myPoses.humans[i].header.stamp;

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    marker.ns = "human_shapes";

	    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	    if(marker.id % 3 ==2){
	      marker.mesh_resource = "package://social_filter/meshes/female1/models/female1.dae";
	      marker.scale.x = 0.4;
	      marker.scale.y = 0.4;
	      marker.scale.z = 0.4;
	    }
	     if(marker.id % 3 ==0){
	       marker.mesh_resource = "package://social_filter/meshes/female1/models/female1.dae";
	      marker.scale.x = 0.4;
	      marker.scale.y = 0.4;
	      marker.scale.z = 0.4;
	      //To translate a little bit
	      marker.pose.position.x = myPoses.humans[i].x;
	      marker.pose.position.y = myPoses.humans[i].y;
	    }
	     if(marker.id % 3 ==1){
	       marker.mesh_resource = "package://social_filter/meshes/female1/models/female1.dae";
	       marker.scale.x = 0.4;
	       marker.scale.y = 0.4;
	       marker.scale.z = 0.4;
	       //To translate a little bit
	       marker.pose.position.x = myPoses.humans[i].x;
	       marker.pose.position.y = myPoses.humans[i].y;
	     }

	    marker.mesh_use_embedded_materials = true;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(myPoses.humans[i].theta + 1.57);

	    ma.markers.push_back(marker);
	  }
      }//if options

      if(options[2]=='1'){  
	/***
	 *Create Markers for each interaction****************************************
	 ***/
	for(uint k=0; k< myList.formation.size(); k++)
	  {
	   
	    visualization_msgs::Marker marker;
 
	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.12;//1.0;
	    marker.scale.y = 0.12;//1.0;
	    marker.scale.z = 0.2;//1.0;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.0f;
	    marker.color.g = 0.0f;
	    marker.color.b = 1.0f;
	    marker.color.a = 0.5;
	    
	    marker.lifetime = ros::Duration(1.0);
	    marker.id = k;//ma.markers.size();

	    // Set the marker action.  Options are ADD and DELETE
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = 0.0;//
	    marker.pose.position.y = 0.0;//
	    marker.pose.position.z = 0.0;

	    // Set the frame ID and timestamp. 
	    marker.header.frame_id = myList.header.frame_id;
	    marker.header.stamp = myList.header.stamp;

	    marker.ns = "interaction_list";
	    marker.type = visualization_msgs::Marker::POINTS;

	    //Set points for the interaction
	    double min_x=myList.formation[k].media_x -(cell_size*int_marker_w/2.0);
	    double min_y=myList.formation[k].media_y -(cell_size*int_marker_h/2.0);
	 
	    // ROS_INFO("print int type %d k:%d",myList.formation[k].type,k);
	    //adding a text for reference
	    text_marker(myList.formation[k].type,k,myList.formation[k].media_x,myList.formation[k].media_y,&ma );

	    for(int m=0; m<=int_marker_h; m++)
	      for(int n=0; n<= int_marker_w; n++)
		{
		  geometry_msgs::Point p;
		  p.x = min_x +  cell_size*n;
		  p.y =  min_y +  cell_size*m;		 
		  p.z = EvalGauss(p.x,p.y, myList.formation[k]);
		  if(p.z > .05)
		    marker.points.push_back(p);

		}

	    //add new marker
	    ma.markers.push_back(marker);



	    //add marker for meeting points
	    //*************************************
	    if( myList.formation[k].type != CIRCULAR &&  myList.formation[k].type != HOI)
	      {
		visualization_msgs::Marker p_marker;
 
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		p_marker.scale.x = 0.2;
		p_marker.scale.y = 0.2;
		p_marker.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		p_marker.color.r = 0.0f;
		p_marker.color.g = 0.0f;
		p_marker.color.b = 0.0;
		p_marker.color.a = 0.0;

		p_marker.lifetime = ros::Duration(1.0);
		p_marker.id = k*1000;

		// Set the marker action.  Options are ADD and DELETE
		p_marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.
		p_marker.pose.position.x = myList.formation[k].meet_points[0];
		p_marker.pose.position.y = myList.formation[k].meet_points[1];
		p_marker.pose.position.z = 0.5;
		
		// Set the frame ID and timestamp. 
		p_marker.header.frame_id = myList.header.frame_id;
		p_marker.header.stamp = myList.header.stamp;

		p_marker.ns = "meet_list";
		p_marker.type = visualization_msgs::Marker::CYLINDER;

		ma.markers.push_back(p_marker);
		p_marker.id = k*1000 +2;
		p_marker.pose.position.x = myList.formation[k].meet_points[2];
		p_marker.pose.position.y = myList.formation[k].meet_points[3];
		ma.markers.push_back(p_marker);
	      }
	    //*************************************

	  }
      }//if options
      if(options[1]=='1'){ 
	/***
	 *Create Markers for each personal space ************************************
	 ***/
	for(uint i=0;i<myPoses.humans.size();i++)
	  {
	    visualization_msgs::Marker marker;
	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.12;//1.0;
	    marker.scale.y = 0.12;//1.0;
	    marker.scale.z = 0.2;//1.0;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1.0f;
	    marker.color.g = 0.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 0.5;

	    marker.lifetime = ros::Duration(1.0);
	    marker.id = ma.markers.size();

	    // Set the marker action.  Options are ADD and DELETE
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = 0.0;// myPoses.humans[i].x;
	    marker.pose.position.y = 0.0;// myPoses.humans[i].y;
	    marker.pose.position.z = 0.0;

	    // Set the frame ID and timestamp.  
	    marker.header.frame_id = myPoses.humans[i].header.frame_id;
	    marker.header.stamp = myPoses.humans[i].header.stamp;

	    marker.ns = "pSpace";
	    marker.type = visualization_msgs::Marker::POINTS;

	    //Set points for the personal space

	    double min_x= myPoses.humans[i].x -(ps_marker_w* cell_size/2.0);
	    double min_y= myPoses.humans[i].y-(ps_marker_h* cell_size/2.0);
	 
	    for(int m=0; m<=ps_marker_h; m++)
	      for(int n=0; n<= ps_marker_w; n++)
		{
		  geometry_msgs::Point p;
		  p.x = min_x +  cell_size*n;
		  p.y =  min_y +  cell_size*m;	
		  p.z = PSpace(p.x, p.y, myPoses.humans[i].x, myPoses.humans[i].y, myPoses.humans[i].theta, pspace_msg);
		  if(p.z > .05)
		    marker.points.push_back(p);	        	
		}

	    //add new marker
	    ma.markers.push_back(marker);

	  }
      }//if options 
      if(options[4]=='1'){ 
	//Create a marker for object
	//******************************************************************************************
	visualization_msgs::Marker marker;
 
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.2f;
	marker.color.b = 0.2f;
	marker.color.a = 0.0;

	marker.lifetime = ros::Duration(1.0);
	marker.id = 99;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x =0.2;
	marker.pose.position.y =15.5;
	marker.pose.position.z = 0;
	geometry_msgs::Quaternion local_orientation=  tf::createQuaternionMsgFromYaw( 1.57);
	marker.pose.orientation= local_orientation;

	// Set the frame ID and timestamp.  
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "obj_shapes";
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://social_filter/meshes/screen/screen.dae";
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;
	marker.mesh_use_embedded_materials = false;
	ma.markers.push_back(marker);
	//******************************************************************************************
      }//if options
      if(options[3]=='1')
	ips_markers(&ma);
      // Publish the markers
      marker_pub.publish(ma);
      ros::spinOnce();
      r.sleep();

    }
 
  return 0;
}



