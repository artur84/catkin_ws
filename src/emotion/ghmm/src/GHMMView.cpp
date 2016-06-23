#include "GHMMView.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <iostream>
#include <iomanip>

using namespace ghmm_ros;

GHMMView::GHMMView(
  const std::string & topic,
  ros::NodeHandle & node
) : topic_( topic ),
    publisher_( node.advertise<visualization_msgs::MarkerArray>( topic, 1000 ) )
{
}

void
GHMMView::operator()( uint32_t id, GHMM::graph_type & graph )
{
  visualization_msgs::MarkerArray markers;

  GHMM::itm_type::node_iterator n;
  GHMM::itm_type::node_iterator nodeEnd;

  uint32_t index = 0;

  for ( boost::tie( n, nodeEnd ) = boost::vertices( graph );
        n != nodeEnd; ++n
  ) {
    GHMM::traits_type::node_data_type & node = graph[*n]; 

    visualization_msgs::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = "ghmm";
    m.id = index++;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = node.centroid[0];
    m.pose.position.y = node.centroid[1];
    m.pose.position.z = node.centroid[2] / ( node.centroid[2] + node.centroid[3] );
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = 0.3 * ( 1 + node.belief * 10 );
    m.scale.y = 0.3 * ( 1 + node.belief * 10 );
    m.scale.z = 0.3 * ( 1 + node.belief * 10 );
    m.color.a = 1.0;
    m.color.r = 0.5;
    m.color.g = node.centroid[2] / ( node.centroid[2] + node.centroid[3] );
    m.color.b = node.centroid[3] / ( node.centroid[2] + node.centroid[3] );
    markers.markers.push_back( m );
//     ROS_INFO("states:%d",markers.markers.size());
  }

  visualization_msgs::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "ghmm";
  m.id = index++;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.03;
  m.color.a = 1.0;
  m.color.r = 0.5;
  m.color.g = 0.5;
  m.color.b = 0.5;
  
  //std::cerr << "\n\n--------------------------\n";

  for ( boost::tie( n, nodeEnd ) = boost::vertices( graph );
        n != nodeEnd; ++n
  ) {
    GHMM::traits_type::node_data_type & n1 = graph[*n]; 
    //std::cerr << n1.centroid << std::endl;

    GHMM::itm_type::out_edge_iterator childEdge;
    GHMM::itm_type::out_edge_iterator childEdgeEnd;

    for ( boost::tie( childEdge, childEdgeEnd ) = boost::out_edges( *n, graph ); 
          childEdge != childEdgeEnd; ++childEdge
    ) {
      const GHMM::node_type & child = boost::target( *childEdge, graph );
      GHMM::traits_type::node_data_type & n2 = graph[child]; 
      //std::cerr << "\t" << n2.centroid << "\t" << graph[*childEdge].probability << std::endl;
      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = n1.centroid[0];
      p1.y = n1.centroid[1];
      p1.z = n1.centroid[2] / ( n1.centroid[2] + n1.centroid[3] );

      p2.x = n2.centroid[0];
      p2.y = n2.centroid[1];
      p2.z = n2.centroid[2] / ( n2.centroid[2] + n2.centroid[3] );
      m.points.push_back( p1 );
      m.points.push_back( p2 );
    }
  }
//   boost::dynamic_properties tmpDP; 
//   boost::write_graphml( std::cerr, graph, boost::get( &GHMM::traits_type::node_data_type::probability, graph ), tmpDP );
  markers.markers.push_back( m );
  publisher_.publish( markers );
}
    
