#include <ros/ros.h>
#include <social_costmap/social_costmap_ros.h>
namespace social_costmap {
  class SocialCostmapNode {
    public:
      SocialCostmapNode(tf::TransformListener& tf) : costmap_ros_("costmap", tf){}
    private:
      SocialCostmapROS costmap_ros_;
  };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_node");

  tf::TransformListener tf(ros::Duration(10));

  social_costmap::SocialCostmapNode* costmap_node;
  costmap_node = new social_costmap::SocialCostmapNode(tf);

  ros::spin();

  delete costmap_node;

  return(0);
}
