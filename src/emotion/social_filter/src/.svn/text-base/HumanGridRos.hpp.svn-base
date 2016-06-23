#ifndef HUMAN_GRID_ROS_HPP
#define  HUMAN_GRIDS_ROS_HPP

#include "ros/ros.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"
#include "social_filter/int_list.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"

class HumanGridRos
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber human_poses_sub_, interaction_list_sub_, global_map_sub_;
	tf::TransformListener tf_listener_;
	ros::Publisher grid_PS_Publisher_, grid_oS_Publisher_,
			grid_complete_publisher_;
	nav_msgs::OccupancyGrid grid_PS_, grid_oS_, grid_complete_, global_map_;
	social_filter::humanPoses human_poses_;
	social_filter::int_list interactions_;
	double x_;//!< Origin of the Social Map in [m], It is the position in in /map of the upper-left corner (WARNING: this is not using the ROS convention)
	double y_;//!< Origin of the Social Map in [m], It is the position in in /map of the upper-left corner (WARNING: this is not using the ROS convention)
	double resolution_; 	//!< The social map resolution [m/cell]
	int steps_;	//!< When using ghmm map only, number of grids in time
	double timeStep_;	//!< When using ghmm map only, time step between grids.
	int width_;	//!< Social map width [cells]
	int height_;	//!< Social map height [cells]
	int max_cost_; //!< Maximum value of the ps grid.
	double ps_sigma_; //!< Standard deviation of the personal space function

	void humanPosesCb_(const social_filter::humanPoses pos);
	void interactionsCb_(const social_filter::int_list i_list);
	void globalMapCb_(const nav_msgs::OccupancyGrid::ConstPtr& msg);

public:
	HumanGridRos();
	~HumanGridRos();
	void init(double x, double y, double resolution, int steps, double timeStep,
			int width, int height, int max_cost, double ps_sigma);
	void initSubs();
	void initGridPublishers();

	void computePsGrid();
	void computeOsGrid();
	void computeCompleteGrid();

	void publishPsGrid();
	void publishOsGrid();
	void publishCompleteGrid();

	int  getIndexFromRC(int r, int c,const  nav_msgs::OccupancyGrid & grid )const;
	void getRelXYFromRC(const int r, const int c, double& x_rel, double& y_rel,const  nav_msgs::OccupancyGrid & grid)const;
	void getAbsXYFromRC(const int r, const int c, double& x_abs, double& y_abs, const  nav_msgs::OccupancyGrid & grid)const;
	void getRCFromRelXY(const double x, const double y, int& r, int& c,const  nav_msgs::OccupancyGrid & grid)const;
	void getRCFromAbsXY(const double x_abs, const double y_abs, int& r, int& c, const  nav_msgs::OccupancyGrid & grid)const;
	void getMapRCFromGridRC(const int rgrid, const int cgrid,  int& rmap,  int& cmap)const;
	void getGridRCFromMapRC(const int rmap, const int cmap,  int& rgrid,  int& cgrid)const;

	bool grid_complete_ready, human_poses_ready;

};

#endif
