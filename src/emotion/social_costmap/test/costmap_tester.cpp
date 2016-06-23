#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <social_costmap/social_costmap_ros.h>

namespace social_costmap {

class CostmapTester : public testing::Test {
  public:
    CostmapTester(tf::TransformListener& tf);
    void checkConsistentCosts();
    void compareCellToNeighbors(social_costmap::SocialCostmap& costmap, unsigned int x, unsigned int y);
    void compareCells(social_costmap::SocialCostmap& costmap, 
        unsigned int x, unsigned int y, unsigned int nx, unsigned int ny);
    virtual void TestBody(){}

  private:
    social_costmap::SocialCostmapROS costmap_ros_;
};

CostmapTester::CostmapTester(tf::TransformListener& tf): costmap_ros_("test_costmap", tf){}

void CostmapTester::checkConsistentCosts(){
  social_costmap::SocialCostmap costmap;

  //get a copy of the costmap contained by our ros wrapper
  costmap_ros_.getCostmapCopy(costmap);

  costmap.saveMap("costmap_test.pgm");

  //loop through the costmap and check for any unexpected drop-offs in costs
  for(unsigned int i = 0; i < costmap.getSizeInCellsX(); ++i){
    for(unsigned int j = 0; j < costmap.getSizeInCellsY(); ++j){
      compareCellToNeighbors(costmap, i, j);
    }
  }
}

void CostmapTester::compareCellToNeighbors(social_costmap::SocialCostmap& costmap, unsigned int x, unsigned int y){
  //we'll compare the cost of this cell with that of its eight neighbors to see if they're reasonable
  for(int offset_x = -1; offset_x <= 1; ++offset_x){
    for(int offset_y = -1; offset_y <= 1; ++offset_y){
      int nx = x + offset_x;
      int ny = y + offset_y;

      //check to make sure that the neighbor cell is a legal one
      if(nx >= 0 && nx < (int)costmap.getSizeInCellsX() && ny >=0 && ny < (int)costmap.getSizeInCellsY()){
        compareCells(costmap, x, y, nx, ny);
      }
    }
  }
}

//for all lethal and inscribed costs, we'll make sure that their neighbors have the cost values we'd expect
void CostmapTester::compareCells(social_costmap::SocialCostmap& costmap, unsigned int x, unsigned int y, unsigned int nx, unsigned int ny){
  double cell_distance = sqrt((x - nx) * (x - nx) + (y - ny) * (y - ny));

  unsigned char cell_cost = costmap.getCost(x, y);
  unsigned char neighbor_cost = costmap.getCost(nx, ny);

  if(cell_cost == social_costmap::LETHAL_OBSTACLE){
    //if the cell is a lethal obstacle, then we know that all its neighbors should have equal or slighlty less cost
    unsigned char expected_lowest_cost = costmap.computeCost(cell_distance);
    EXPECT_TRUE(neighbor_cost >= expected_lowest_cost || (cell_distance > costmap.cell_inflation_radius_ && neighbor_cost == social_costmap::FREE_SPACE));
  }
  else if(cell_cost == social_costmap::INSCRIBED_INFLATED_OBSTACLE){
    //the furthest valid distance from an obstacle is the inscribed radius plus the cell distance away
    double furthest_valid_distance = costmap.cell_inscribed_radius_ + cell_distance + 1;
    unsigned char expected_lowest_cost = costmap.computeCost(furthest_valid_distance);
    if(neighbor_cost < expected_lowest_cost){
      ROS_ERROR("Cell cost (%d, %d): %d, neighbor cost (%d, %d): %d, expected lowest cost: %d, cell distance: %.2f, furthest valid distance: %.2f",
          x, y, cell_cost, nx, ny, neighbor_cost, expected_lowest_cost, cell_distance, furthest_valid_distance);
      ROS_ERROR("Cell: (%d, %d), Neighbor: (%d, %d)", x, y, nx, ny);
      costmap.saveMap("failing_costmap.pgm");
    }
    EXPECT_TRUE(neighbor_cost >= expected_lowest_cost || (furthest_valid_distance > costmap.cell_inflation_radius_ && neighbor_cost == social_costmap::FREE_SPACE));
  }
}
};

social_costmap::CostmapTester* map_tester = NULL;

TEST(CostmapTester, checkConsistentCosts){
  map_tester->checkConsistentCosts();
}

void testCallback(const ros::TimerEvent& e){
  int test_result = RUN_ALL_TESTS();
  ROS_INFO("gtest return value: %d", test_result);
  ros::shutdown();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_tester_node");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  tf::TransformListener tf(ros::Duration(10));
  map_tester = new social_costmap::CostmapTester(tf);

  double wait_time;
  private_nh.param("wait_time", wait_time, 30.0);
  ros::Timer timer = n.createTimer(ros::Duration(wait_time), testCallback);

  ros::spin();

  return(0);
}
