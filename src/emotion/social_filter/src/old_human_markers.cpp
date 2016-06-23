#include "ros/ros.h"
#include "std_msgs/String.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"
#include "social_filter/int_list.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_listener.h"
#include "social_filter/social_fcns.h"

/**
 * This node permits to visualize humans and interactions in RVIZ
 */
social_filter::humanPoses myPoses;
social_filter::int_list myList;

double cell_size= .1; //in meters
int int_marker_w=60;
int int_marker_h=60;
int ps_marker_w=50;
int ps_marker_h=50;

void peopleCallback(const social_filter::humanPoses pos)
{
  myPoses.humans.clear();
  for(uint i=0;i<pos.humans.size();i++){
    myPoses.humans.push_back(pos.humans[i]);
  }
}

void intCallback(const social_filter::int_list i_list)
{
  myList.header.frame_id = i_list.header.frame_id;
  myList.header.stamp = i_list.header.stamp;
  myList.formation.clear();
  for(uint i=0;i<i_list.formation.size();i++){
    myList.formation.push_back(i_list.formation[i]);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "humanMarkers");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("human_poses", 1, peopleCallback);
  ros::Subscriber sub_int=n.subscribe("interaction_list", 1, intCallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("human_markers", 1);
  ros::Rate r(10);

  while (ros::ok()){
    visualization_msgs::MarkerArray ma;

    /***
      *Create Markers for each human
      ***/
    for(uint i=0;i<myPoses.humans.size();i++){
      visualization_msgs::Marker marker;

      marker.scale.x = 0.8;
      marker.scale.y = 0.8;
      marker.scale.z = 1;

      marker.lifetime = ros::Duration(1.0);
      marker.id = i;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = myPoses.humans[i].x;
      marker.pose.position.y = myPoses.humans[i].y;
      marker.pose.position.z = 0;

      geometry_msgs::Quaternion local_orientation =  tf::createQuaternionMsgFromYaw(myPoses.humans[i].theta + 1.57);
      marker.pose.orientation= local_orientation;

      // Set the frame ID and timestamp.
      marker.header.frame_id = myPoses.humans[i].header.frame_id;
      marker.header.stamp = myPoses.humans[i].header.stamp;

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "human_shapes";

      marker.type = visualization_msgs::Marker::CYLINDER;

      if(marker.id > 8){
        marker.color.r = 0.36;
        marker.color.g = 0.20;
        marker.color.b = 0.4;
        marker.color.a = 1.0;
      }
      else{
        marker.color.r = 0.96;
        marker.color.g = 0.47;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      }
        marker.mesh_use_embedded_materials = true;
        ma.markers.push_back(marker);
    }

    // Publish the markers
    marker_pub.publish(ma);
    ros::spinOnce();
    r.sleep();
  }
 return 0;
}


//       /***
//       *Create Markers for each human
//       ***/
//     for(uint i=0;i<myPoses.humans.size();i++){
//       visualization_msgs::Marker marker;
// //       ROS_INFO("marker number:%d",i);
//
//       // Set the color -- be sure to set alpha to something non-zero!
// //       marker.color.r = 0.0f;
// //       marker.color.g = 1.0f;
// //       marker.color.b = 0.0f;
// //       marker.color.a = 1.0;
//       marker.color.r = 0.0f;
//       marker.color.g = 0.0f;
//       marker.color.b = 0.0f;
//       marker.color.a = 0.0;
//
//       marker.lifetime = ros::Duration(1.0);
//       marker.id = i;
//
//       // Set the marker action.  Options are ADD and DELETE
//       marker.action = visualization_msgs::Marker::ADD;
//
//       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//       marker.pose.position.x = myPoses.humans[i].x;
//       marker.pose.position.y = myPoses.humans[i].y;
//       marker.pose.position.z = 0;
//
//       geometry_msgs::Quaternion local_orientation=  tf::createQuaternionMsgFromYaw(myPoses.humans[i].theta + 1.57);
//       //TODO: why 1.57?
//       marker.pose.orientation= local_orientation;
//
//       // Set the frame ID and timestamp.
//       marker.header.frame_id = myPoses.humans[i].header.frame_id;
//       marker.header.stamp = myPoses.humans[i].header.stamp;
//
//       // Set the namespace and id for this marker.  This serves to create a unique ID
//       // Any marker sent with the same namespace and id will overwrite the old one
//       marker.ns = "human_shapes";
//       marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//
//       if(marker.id % 2 ==0){
//         marker.mesh_resource = "package://social_filter/meshes/female1/models/female1.dae";
//         marker.scale.x = 0.01;
//         marker.scale.y = 0.01;
//         marker.scale.z = 0.01;
//       }
//       else{
//         marker.mesh_resource = "package://social_filter/meshes/man1/models/man1.dae";
//         marker.scale.x = 0.04;
//         marker.scale.y = 0.04;
//         marker.scale.z = 0.04;
//         //To translate a little bit
// //         marker.pose.position.x = myPoses.humans[i].x + 0.2;
// //         marker.pose.position.y = myPoses.humans[i].y - 0.1;
//         marker.pose.position.x = myPoses.humans[i].x;
//         marker.pose.position.y = myPoses.humans[i].y;
//       }
//         marker.mesh_use_embedded_materials = true;
//         ma.markers.push_back(marker);
//     }


//     /***
//       *Create Markers for each interaction
//       ***/
//     for(uint k=0; k< myList.formation.size(); k++){
//       visualization_msgs::Marker marker;
//
//       // Set the scale of the marker -- 1x1x1 here means 1m on a side
//       marker.scale.x = 0.05;//1.0;
//       marker.scale.y = 0.05;//1.0;
//       marker.scale.z = 0.05;//1.0;
//
//       // Set the color -- be sure to set alpha to something non-zero!
//       marker.color.r = 0.0f;
//       marker.color.g = 0.0f;
//       marker.color.b = 1.0f;
//       marker.color.a = 1.0;
//
//       marker.lifetime = ros::Duration(1.0);
//       marker.id = ma.markers.size();
//
//       // Set the marker action.  Options are ADD and DELETE
//       marker.action = visualization_msgs::Marker::ADD;
//
//       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//       marker.pose.position.x = 0.0;// myList.formation[k].media_x;
//       marker.pose.position.y = 0.0;//myList.formation[k].media_y;
//       marker.pose.position.z = 0.0;
//
//       // Set the frame ID and timestamp.
//       marker.header.frame_id = myList.header.frame_id;
//       marker.header.stamp = myList.header.stamp;
//
//       marker.ns = "interaction_list";
//       marker.type = visualization_msgs::Marker::POINTS;
//
//       //Set points for the interaction
//       double min_x=myList.formation[k].media_x -(cell_size*int_marker_w/2.0);
//       double min_y=myList.formation[k].media_y -(cell_size*int_marker_h/2.0);
//
//       for(int m=0; m<=int_marker_h; m++){
//         for(int n=0; n<= int_marker_w; n++){
//             geometry_msgs::Point p;
//             p.x = min_x +  cell_size*n;
//             p.y =  min_y +  cell_size*m;
//
//             p.z = EvalGauss(p.x,p.y, myList.formation[k]);
//             if(p.z > .05)
//             marker.points.push_back(p);
//         }
//       }
//
//       //add new marker
//       ma.markers.push_back(marker);
//     }

      /***
       *Create Markers for each personal space
       ***/
//       for(uint i=0;i<myPoses.humans.size();i++)
// 	{
// 	  visualization_msgs::Marker marker;
// 	  // Set the scale of the marker -- 1x1x1 here means 1m on a side
// 	  marker.scale.x = 0.05;//1.0;
// 	  marker.scale.y = 0.05;//1.0;
// 	  marker.scale.z = 0.05;//1.0;
//
// 	  // Set the color -- be sure to set alpha to something non-zero!
// 	  marker.color.r = 1.0f;
// 	  marker.color.g = 0.0f;
// 	  marker.color.b = 0.0f;
// 	  marker.color.a = 1.0;
//
// 	  marker.lifetime = ros::Duration(1.0);
// 	  marker.id = ma.markers.size();
//
// 	  // Set the marker action.  Options are ADD and DELETE
// 	  marker.action = visualization_msgs::Marker::ADD;
//
// 	  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// 	  marker.pose.position.x = 0.0;// myPoses.humans[i].x;
// 	  marker.pose.position.y = 0.0;// myPoses.humans[i].y;
// 	  marker.pose.position.z = 0.0;
//
// 	  // Set the frame ID and timestamp.
// 	  marker.header.frame_id = myPoses.humans[i].header.frame_id;
// 	  marker.header.stamp = myPoses.humans[i].header.stamp;
//
// 	  marker.ns = "pSpace";
// 	  marker.type = visualization_msgs::Marker::POINTS;
//
// 	  //Set points for the personal space
//
// 	  double min_x= myPoses.humans[i].x -(ps_marker_w* cell_size/2.0);
// 	  double min_y= myPoses.humans[i].y-(ps_marker_h* cell_size/2.0);
//
// 	  for(int m=0; m<=ps_marker_h; m++)
// 	    for(int n=0; n<= ps_marker_w; n++)
// 	      {
// 		geometry_msgs::Point p;
// 		p.x = min_x +  cell_size*n;
// 		p.y =  min_y +  cell_size*m;
//
// 		p.z = PSpace(p.x, p.y, myPoses.humans[i].x, myPoses.humans[i].y, myPoses.humans[i].theta);
// 		if(p.z > .05)
// 		marker.points.push_back(p);
// 	      }
//
// 	  //add new marker
// 	  ma.markers.push_back(marker);
//
// 	}


