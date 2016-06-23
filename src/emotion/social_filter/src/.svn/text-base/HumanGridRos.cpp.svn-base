#include "HumanGridRos.hpp"
#include "social_fcns.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

HumanGridRos::HumanGridRos()
{
	grid_complete_ready = false;
	human_poses_ready = false;
	HumanGridRos::initSubs();
	HumanGridRos::initGridPublishers();
}

HumanGridRos::~HumanGridRos()
{
}

	/**
	* @param x: [m] X position of the lower-left pixel of the grid in the /map reference frame
	* @param y: [m] Y position of the lower-left pixel of the grid in the /map reference frame
	* @param resolution: [m/cells] The map resolution.
	* @param height: [cells]
	* @param steps:  When using ghmm map only, the number of grids in time
	*/
void HumanGridRos::init(double x, double y, double resolution, int steps,
		double timeStep, int width, int height, int max_cost, double ps_sigma)
{

	ROS_INFO("Social Grid Initialized with following parameters:");
	x_ = x;
	y_ = y;
	resolution_ = resolution;
	ROS_INFO("cellSize: %f", resolution_);
	steps_ = steps;
	timeStep_ = timeStep;
	ROS_INFO("timeStep: %f", timeStep_);
	width_ = width;
	ROS_INFO("width: %d", width_);
	height_ = height;
	ROS_INFO("height: %d", height_);
	max_cost_ = max_cost;
	ps_sigma_ = ps_sigma;
	ROS_INFO("max_cost: %d", max_cost_);
	ROS_INFO("ps_sigma: %f", ps_sigma_);
	//Init Personal Space grid information
	grid_PS_.data.resize(width_ * height_);
	grid_PS_.info.resolution = resolution_;
	grid_PS_.info.width = width_;
	grid_PS_.info.height = height_;
	grid_PS_.info.origin.position.x = x_;
	grid_PS_.info.origin.position.y = y_;
	grid_PS_.info.origin.position.z = 0.0;
	grid_PS_.header.frame_id = "/map";
	//Init o space grid information
	grid_oS_.data.resize(width_ * height_);
	grid_oS_.info.resolution = resolution_;
	grid_oS_.info.width = width_;
	grid_oS_.info.height = height_;
	grid_oS_.info.origin.position.x = x_;
	grid_oS_.info.origin.position.y = y_;
	grid_oS_.info.origin.position.z = 0.0;
	grid_oS_.header.frame_id = "/map";

}

void HumanGridRos::humanPosesCb_(const social_filter::humanPoses pos)
{
	//ROS_INFO("IN human_poses_ callback");
	human_poses_.humans.clear();
	social_filter::humanPose human1;
	for (unsigned int i = 0; i < pos.humans.size(); i++)
	{
		try
		{
			tf_listener_.waitForTransform("/map", pos.humans[i].header.frame_id,
					ros::Time(0), ros::Duration(3.0));
			//this is to have humans in map coordinates
			transformPosetoMap(&tf_listener_, pos.humans[i], &human1);
		} catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		human1.header.stamp = pos.humans[i].header.stamp;
		human1.header.frame_id = pos.humans[i].header.frame_id;
		human1.linear_velocity = pos.humans[i].linear_velocity;
		human1.angular_velocity = pos.humans[i].angular_velocity;
		human1.id = pos.humans[i].id;
		human_poses_.humans.push_back(human1);
	}
	human_poses_ready = true;
}

void HumanGridRos::interactionsCb_(const social_filter::int_list i_list)
{
	interactions_.header.frame_id = i_list.header.frame_id;
	interactions_.header.stamp = i_list.header.stamp;
	interactions_.formation.clear();
	for (int i = 0; i < i_list.formation.size(); i++)
	{
		interactions_.formation.push_back(i_list.formation[i]);
	}
}
//Due to the way the map_server node works, this callback happens only once when the subscriber is created
void HumanGridRos::globalMapCb_(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("Complete grid initialized in the callback.");
	grid_complete_.header.frame_id = msg->header.frame_id;
	grid_complete_.info.origin.position.x = msg->info.origin.position.x;
	grid_complete_.info.origin.position.y = msg->info.origin.position.y;
	ROS_INFO("grid_complete_.info.origin.position.x= %f", grid_complete_.info.origin.position.x);
	ROS_INFO("grid_complete_.info.origin.position.y= %f", grid_complete_.info.origin.position.y);
	grid_complete_.info.width = msg->info.width;
	grid_complete_.info.height = msg->info.height;
	grid_complete_.info.resolution = msg->info.resolution;
	grid_complete_.data.resize(
			grid_complete_.info.width * grid_complete_.info.height);

	global_map_.header.frame_id = msg->header.frame_id;
	global_map_.info.origin.position.x = msg->info.origin.position.x;
	global_map_.info.origin.position.y = msg->info.origin.position.y;
	global_map_.info.width = msg->info.width;
	global_map_.info.height = msg->info.height;
	global_map_.info.resolution = msg->info.resolution;
	global_map_.data.resize(global_map_.info.width * global_map_.info.height);
	for (unsigned int i = 0;
			i < global_map_.info.width * global_map_.info.height; i++)
	{
		global_map_.data[i] = msg->data[i];
	}
	ROS_INFO("cellSize: %f", grid_complete_.info.resolution);
	ROS_INFO("width: %d", grid_complete_.info.width);
	ROS_INFO("height: %d", grid_complete_.info.height);

	grid_complete_ready = true;
}

void HumanGridRos::initSubs()
{

	human_poses_sub_ = nh_.subscribe("human_poses", 10,
			&HumanGridRos::humanPosesCb_, this);
	interaction_list_sub_ = nh_.subscribe("interaction_list", 10,
			&HumanGridRos::interactionsCb_, this);
	global_map_sub_ = nh_.subscribe("/map", 1,
			&HumanGridRos::globalMapCb_, this);
}

void HumanGridRos::initGridPublishers()
{
	grid_PS_Publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("personal_grid",
			1);
	grid_oS_Publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
			"interaction_grid", 1);
	grid_complete_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
			"social_grid", 1);

}

//this function is used to publish personal spaces for humans
void HumanGridRos::computePsGrid()
{
	double y = y_ + resolution_ / 2.0;
	double ps_cost;
	for (int j = 0; j < height_; ++j)
	{
		double x = x_ + resolution_ / 2.0;
		for (int i = 0; i < width_; ++i)
		{
			double p_coll = 1.0;
			for (int k = 0; k < human_poses_.humans.size(); k++)
				p_coll *= (1.0
						- PSpace(x, y, human_poses_.humans[k].x,
								human_poses_.humans[k].y,
								human_poses_.humans[k].theta, ps_sigma_));
			//ROS_INFO("hoal %f", ps_sigma_);
			ps_cost = int((1.0 - p_coll) * 100 + 0.5);
			//Marks as occupied regions with a social cost value higher than max_cost_
			if (ps_cost >= max_cost_)
				grid_PS_.data[j * width_ + i]= 100;
			//Otherwise mark them as free
			else
				grid_PS_.data[j * width_ + i]=0;

			x += resolution_;
		}
		y += resolution_;
	}
}

//this function is used to publish personal spaces for humans
void HumanGridRos::publishPsGrid()
{
	HumanGridRos::computePsGrid();
	grid_PS_.header.stamp = ros::Time::now();
	grid_PS_Publisher_.publish(grid_PS_);
//  ROS_INFO("Published PS grid");
}

/***
 * computeOsGrid
 */
void HumanGridRos::computeOsGrid()
{
	double y = y_ + resolution_ / 2.0; //Why 2?= is to put "y" in the center of the cell
	for (int j = 0; j < height_; ++j)
	{
		double x = x_ + resolution_ / 2.0;
		for (int i = 0; i < width_; ++i)
		{
			double p_coll = 1.0;
			for (int k = 0; k < interactions_.formation.size(); k++)
				p_coll *= (1.0 - EvalGauss(x, y, interactions_.formation[k]));
			grid_oS_.data[j * width_ + i] = int((1.0 - p_coll) * 100 + 0.5);
			x += resolution_;
		}
		y += resolution_;
	}
}

/***
 * publishOsGrid
 */
void HumanGridRos::publishOsGrid()
{
	HumanGridRos::computeOsGrid();
	grid_oS_.header.stamp = ros::Time::now();
	grid_oS_Publisher_.publish(grid_oS_);
//    ROS_INFO("Published oS grid");

}

/***
 * computeCompleteGrid: PS+OS+map
 */
void HumanGridRos::computeCompleteGrid()
{
	HumanGridRos::computePsGrid();
	HumanGridRos::computeOsGrid();
	//Copy global map data
	for (int i = 0;
			i < global_map_.info.width * global_map_.info.height; i++)
	{
		grid_complete_.data[i] = global_map_.data[i];
	}
	int r_map, c_map, map_index, ps_index;
	int h = grid_PS_.info.height;
	int w = grid_PS_.info.width;
	//Adding PS grid data
	for (int r = 0; r < grid_PS_.info.height; ++r)
	{

		//ROS_INFO("J=%d", J);
		for (int c = 0; c < grid_PS_.info.width; ++c)
		{
			getMapRCFromGridRC(r,c,r_map,c_map);
			map_index = getIndexFromRC(r_map, c_map, global_map_);
			//ps_index = getIndexFromRC(r,c,grid_PS_);

			//ROS_INFO("I=%d, J=%d", I,J);
			if (c== 0)
			{
				ROS_INFO("r=%d, c= %d",r, c);
				ROS_INFO("r_map=%d, c_map= %d, map_index= %d, ps_index =%d",r_map, c_map, map_index, (int)(r)*w+c);
			}

			grid_complete_.data[map_index] = grid_PS_.data[(int)(r)*w+c]+grid_oS_.data[(int)(r)*w+c]
					+ global_map_.data[map_index];

		}
	}
}

/***
 * publishCompleteGrid
 */
void HumanGridRos::publishCompleteGrid()
{

	HumanGridRos::computeCompleteGrid();
	grid_complete_.info.map_load_time = ros::Time::now();
	grid_complete_.header.stamp = ros::Time::now();
	grid_complete_publisher_.publish(grid_complete_);

}


void HumanGridRos::getGridRCFromMapRC(const int rmap, const int cmap,  int& rgrid,  int& cgrid)const{
	assert ( rmap < global_map_.info.height);
	assert ( cmap < global_map_.info.width);
	double Oxg = grid_PS_.info.origin.position.x;//origin of PS in x
	double Oyg = grid_PS_.info.origin.position.y;
	double resg = grid_PS_.info.resolution;//resolution of ps grid
	double resm = global_map_.info.resolution;//resolution of global map grid
	double hm=global_map_.info.height; //height of global map
	double hg = grid_PS_.info.height; //height of ps grid

	rgrid = (int)round(hg - (resm/resg)*(hm-rmap)-Oyg/resg);
	cgrid = (int)round((cmap*resm - Oxg)/resg);
}

void HumanGridRos::getMapRCFromGridRC(const int rgrid, const int cgrid,  int& rmap,  int& cmap)const{
	assert ( rgrid < grid_PS_.info.height);
	assert ( cgrid < grid_PS_.info.width);
//	double x_abs, y_abs;
//	int r,c;
//	getAbsXYFromRC(rgrid,cgrid,x_abs,y_abs,grid_PS_);
//	getRCFromAbsXY(x_abs,y_abs,r,c, global_map_);
//	if ((r>=0 && r< global_map_.info.height) && (c>=0 && c < global_map_.info.width ))
//	{
//		rmap=r;
//		cmap=c;
//	}
//	else
//	{
//		//TODO: Maybe I should change this to raise an error and finish the program?
//		ROS_WARN("Required points of PS GRID don't exist in global_map, putting it to 0");
//		rmap=0;
//		cmap=0;
//	}

	double Oxg =(double) grid_PS_.info.origin.position.x;//origin of PS in x
	double Oyg =(double) grid_PS_.info.origin.position.y;
	double resg =(double) grid_PS_.info.resolution;//resolution of ps grid
	double resm =(double) global_map_.info.resolution;//resolution of global map grid
	double hm=(double) global_map_.info.height; //height of global map
	double hg =(double) grid_PS_.info.height; //height of ps grid
	//TODO: print this to check why it is always the same value.

	rmap = (int)round((resg*rgrid + Oyg)/resm);
	cmap = (int)round((cgrid*resg + Oxg)/resm);
}

int HumanGridRos::getIndexFromRC(int r, int c, const  nav_msgs::OccupancyGrid & grid)const {
	assert ( r < grid.info.height);
	assert ( c < grid.info.width);
	return (r * grid.info.width + c);
};

void HumanGridRos::getRelXYFromRC(const int r, const int c,  double& x_rel, double& y_rel, const  nav_msgs::OccupancyGrid & grid)const{
	//Assumes ros convention where the XY origin is in the bottom-left corner and the RC is in the upper-left.
	std::cout <<"r= "<< r << ", c= " << c << "  ";
	assert ( r < grid.info.height);
	assert ( c < grid.info.width);
	double height=(double) grid.info.height;
	x_rel = grid.info.resolution * (c +0.5);
	y_rel = grid.info.resolution * (height - (r +0.5));
}

void HumanGridRos::getAbsXYFromRC(const int r, const int c,  double& x_abs, double& y_abs, const  nav_msgs::OccupancyGrid & grid)const{
	//Assumes ros convention where the cell
	double x_relative, y_relative;//x,y coordinates in the grid reference frame.
	//double st, ct;//sin theta, cos theta
	double h = (double)grid.info.height;
	std::cout << "abs\n";
	getRelXYFromRC(r, c, x_relative, y_relative, grid);
	//sincos(0 , &st, &ct); //TODO: generalize this to consider angles other than just zero
	//x_abs = x_relative*ct - y_relative*st + grid.info.origin.position.x;
	//y_abs = x_relative*st + y_relative*ct + grid.info.origin.position.y;
	x_abs = x_relative + grid.info.origin.position.x;
	y_abs = y_relative + grid.info.origin.position.y;
}
void HumanGridRos::getRCFromRelXY(const double x_rel, const double y_rel, int& r, int& c, const  nav_msgs::OccupancyGrid & grid)const {
	double h = (double)grid.info.height;
	c = (int)round((x_rel / grid.info.resolution) - 0.5);
	r = (int)round(h-0.5-y_rel/grid.info.resolution);
};

void HumanGridRos::getRCFromAbsXY(const double x_abs, const double y_abs, int& r, int& c, const  nav_msgs::OccupancyGrid & grid)const {

	double x_relative, y_relative;
	//double st, ct;
	//sincos(0 , &st, &ct); //TODO: generalize this to consider angles other than just zero
	x_relative = x_abs - grid.info.origin.position.x;
	y_relative = y_abs - grid.info.origin.position.y;
	getRCFromRelXY( x_relative,  y_relative, r, c,  grid);
};


int main(int argc, char **argv)
{

	ros::init(argc, argv, "humanGrids");
	ros::NodeHandle private_nh("~");// Private node handler to read ros parameters
	HumanGridRos h_grid;

	double x,y,resolution,time_step, ps_sigma;
	int steps,width, height, max_cost;
	//Reading ROS params
	private_nh.param("x", x, 0.0);
	private_nh.param("y", y, 0.0);
	private_nh.param("resolution", resolution, 0.15);
	private_nh.param("steps", steps, 10);
	private_nh.param("time_step", time_step, 0.5);
	private_nh.param("width", width, 35);
	private_nh.param("height", height,35);
	private_nh.param("max_cost", max_cost ,120);
	private_nh.param("ps_sigma", ps_sigma , 0.42);
	//ROS_INFO("ps_sigma==,%f", ps_sigma);

	h_grid.init(x, y, resolution, steps, time_step, width, height, max_cost, ps_sigma);

	ros::Rate r(5);

	while (ros::ok())
	{
		if (h_grid.grid_complete_ready && h_grid.human_poses_ready)
		{
			h_grid.publishPsGrid();
			h_grid.publishCompleteGrid();
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
