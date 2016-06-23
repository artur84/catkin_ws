//=======================================================================
// Copyright 2001 Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee, 
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace boost;
using namespace std;

int
main(int, char *[])
{
  //OutEdgeList, VertexList, Directed 
  typedef adjacency_list<listS, vecS, undirectedS, no_property, property<
      edge_weight_t, int> > graph_t;
  typedef graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
  typedef graph_traits<graph_t>::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> Edge;

  string line;


  ifstream myfile;
  myfile.open(
      "/home/arturo/ros_works/pal/teams/emotion/user_intentiosdafns/docs/inria_goals.yaml",
      ios::out);
  if (myfile.is_open())
    {
      while (myfile.good())
        {
//myfile >> data;
          getline(myfile,line);//theChar now holds one char (can be anything including spaces)
          cout << "data read " << endl;
          cout << line << endl;
        }
      myfile.close();
    }

  else
    cout << "Unable to open file";

  const int num_nodes = 4;
  enum nodes
  {
    A, B, C, D, E, F
  };
  char name[] = "ABCDEF";
  Edge edge_array[] =
    { Edge(A, B), Edge(A, C), Edge(B, E), Edge(C, D), Edge(D, F), Edge(E, F) };
  geometry_msgs::PoseStamped goal_msg;
  std::vector<geometry_msgs::PoseStamped> goals;
  //This goals must follow the same order than nodes
  ros::Time t(1.0);
  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 0.0;
  goal_msg.pose.position.y = 0.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 1.0;
  goal_msg.pose.position.y = 1.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 2.0;
  goal_msg.pose.position.y = 2.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 3.0;
  goal_msg.pose.position.y = 3.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 4.0;
  goal_msg.pose.position.y = 4.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  goal_msg.header.stamp = t;
  goal_msg.pose.position.x = 5.0;
  goal_msg.pose.position.y = 5.0;
  goal_msg.pose.position.z = 0.;
  goals.push_back(goal_msg);

  int weights[] =
    { 1, 1, 2, 1, 4, 2 };
  int num_arcs = sizeof(edge_array) / sizeof(Edge);

  graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
  property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);

  std::vector<vertex_descriptor> p(num_vertices(g));
  std::vector<int> d(num_vertices(g));
  //here we are telling which node would be the source vertex
  vertex_descriptor s = vertex(A, g);
  vertex_descriptor goal = vertex(F, g);

  dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));

  // std::cout << "distances and parents:" << std::endl;
  // graph_traits < graph_t >::vertex_iterator vi, vend;
  // for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
  //   std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
  //   std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
  //     endl;
  // }
  // std::cout << std::endl;

  std::cout << "short distance to :" << name[goal] << std::endl;
  vertex_descriptor parent, current;
  parent = goal;
  do
    {
      current = parent;
      std::cout << " " << name[current];
      std::cout << " (" << goals[current].pose.position.x << ","
          << goals[current].pose.position.y << ")" << std::endl;
      parent = p[current];
    }
  while (parent != s && current != parent);

  //this is to generate the graph in dot format
  //to generate ps type:
  //dot -Tps dijkstra-eg.dot -o graph1.ps
  std::ofstream dot_file("/home/jorge/figs/dijkstra-eg.dot");

  dot_file << "graph D {\n"
  // << "  rankdir=LR\n"
      << "  size=\"4,3\"\n" << "  ratio=\"fill\"\n"
      << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

  graph_traits<graph_t>::edge_iterator ei, ei_end;
  for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
    {
      graph_traits<graph_t>::edge_descriptor e = *ei;
      graph_traits<graph_t>::vertex_descriptor u = source(e, g), v = target(e,
          g);
      dot_file << name[u] << " -- " << name[v] << "[label=\"" << get(weightmap,
          e) << "\"";
      //if (p[v] == s)
      dot_file << ", color=\"black\"";
      // else
      // dot_file << ", color=\"grey\"";
      dot_file << "]";
    }
  dot_file << "}";
  return (EXIT_SUCCESS);
}
