#include <social_costmap/social_grid.h>
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

SocialGrid::SocialGrid()
{
	grid_complete_ready = false;
	human_poses_ready = false;
	SocialGrid::initSubs();
	SocialGrid::initGridPublishers();
}

SocialGrid::~SocialGrid()
{
}

void SocialGrid::init(double x, double y, double resolution, int steps,
		double timeStep, int width, int height, int max_ps_cost,
		int max_os_cost, social_filter::humanSocialSpace human_social_space)
{
	/** Init the grid
	 * @param x: [m] X position of the lower-left pixel of the grid in the /map reference frame
	 * @param y: [m] Y position of the lower-left pixel of the grid in the /map reference frame
	 * @param resolution: [m/cells] The map resolution.
	 * @param steps:  When using ghmm map only, the number of grids in time
	 * @param timeStep: [seconds] when using ghmm map only, the time between grids.
	 * @param width: [cells]
	 * @param height: [cells]
	 *
	 */

	ROS_INFO("Social Grid Initialized with following parameters:");
	x_ = x;
	y_ = y;
	resolution_ = resolution;
	ROS_INFO("cellSize: %f", resolution_);
	steps_ = steps;
	time_step_ = timeStep;
	ROS_INFO("timeStep: %f", time_step_);
	width_ = width;
	ROS_INFO("width: %d", width_);
	height_ = height;
	ROS_INFO("height: %d", height_);
	max_ps_cost_ = max_ps_cost;
	human_socialSpace_ = human_social_space;
	ROS_INFO("max_ps_cost: %d", max_ps_cost_);
	ROS_INFO("ps_sigma: %f", ps_sigma_);
	max_os_cost_ = max_os_cost;
	ROS_INFO("max_os_cost: %d", max_os_cost_);
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

void SocialGrid::humanPosesCb_(const social_filter::humanPoses pos)
{
	/** Callback activated when a human pose from humanproc or kinect_human proc is published.
	 * @param pos: social_filter::humanPoses; a vector conatining position an velocity for each detected human.
	 */
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

void SocialGrid::interactionsCb_(const social_filter::int_list i_list)
{
	/** Callback activated when a list of interactions is received (normally published by fform node)
	 * @param i_list: social_filter::humanPoses; List of iteractions
	 */
	interactions_.header.frame_id = i_list.header.frame_id;
	interactions_.header.stamp = i_list.header.stamp;
	interactions_.formation.clear();
	for (uint32_t i = 0; i < i_list.formation.size(); i++)
	{
		interactions_.formation.push_back(i_list.formation[i]);
	}
}

void SocialGrid::globalMapCb_(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	/*** @brief Gets the "global" map from ROS map server
	 *
	 * Due to the way the map_server node works, this callback happens only once when the subscriber is created
	 *
	 */
	ROS_INFO("Complete grid initialized in the callback.");
	grid_complete_.header.frame_id = msg->header.frame_id;
	grid_complete_.info.origin.position.x = msg->info.origin.position.x;
	grid_complete_.info.origin.position.y = msg->info.origin.position.y;
	ROS_INFO("grid_complete_.info.origin.position.x= %f",
			grid_complete_.info.origin.position.x);
	ROS_INFO("grid_complete_.info.origin.position.y= %f",
			grid_complete_.info.origin.position.y);
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

void SocialGrid::initSubs()
{

	human_poses_sub_ = nh_.subscribe("human_poses", 10,
			&SocialGrid::humanPosesCb_, this);
	interaction_list_sub_ = nh_.subscribe("interaction_list", 10,
			&SocialGrid::interactionsCb_, this);
	global_map_sub_ = nh_.subscribe("/map", 1, &SocialGrid::globalMapCb_, this);
}

void SocialGrid::initGridPublishers()
{
	grid_PS_Publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("personal_grid",
			1);
	grid_oS_Publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
			"interaction_grid", 1);
	grid_complete_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(
			"social_grid", 1);

}

void SocialGrid::computePsGrid()
{
	/***
	 * Compute the personal space around people received in the human_poses list.
	 */
	double y = y_ + resolution_ / 2.0;
	double ps_cost;
	for (int j = 0; j < height_; ++j)
	{
		double x = x_ + resolution_ / 2.0;
		for (int i = 0; i < width_; ++i)
		{
			double p_coll = 1.0;
			for (uint32_t k = 0; k < human_poses_.humans.size(); k++)
				p_coll *= (1.0
						- PSpace(x, y, human_poses_.humans[k].x,
								human_poses_.humans[k].y,
								human_poses_.humans[k].theta, human_socialSpace_));
			ps_cost = int((1.0 - p_coll) * 100); //It should be ps_cost = int((1.0 - p_coll) * 100 + 0.5) but given the next step Idon't need to be so precise.;
			//Marks regions with a social cost value higher than max_ps_cost_ as occupied.
			if (ps_cost >= max_ps_cost_)
				grid_PS_.data[j * width_ + i] = 100;
			//Otherwise mark them as free
			else
				grid_PS_.data[j * width_ + i] = 0;

			x += resolution_;
		}
		y += resolution_;
	}
}

//this function is used to publish personal spaces for humans
void SocialGrid::publishPsGrid()
{

	SocialGrid::computePsGrid();
	grid_PS_.header.stamp = ros::Time::now();
	grid_PS_Publisher_.publish(grid_PS_);
//  ROS_INFO("Published PS grid");
}

void SocialGrid::computeOsGrid()
{
	/***
	 * Computes the grid containing interaction spaces marked as occupied regions
	 */
	int os_cost;
	double y = y_ + resolution_ / 2.0; //Why 2?= is to put "y" in the center of the cell
	for (int j = 0; j < height_; ++j)
	{
		double x = x_ + resolution_ / 2.0;
		for (int i = 0; i < width_; ++i)
		{
			double p_coll = 1.0;
			for (uint32_t k = 0; k < interactions_.formation.size(); k++)
				p_coll *= (1.0 - EvalGauss(x, y, interactions_.formation[k]));

			os_cost = int((1.0 - p_coll) * 100 + 0.5);
			//std::cout << os_cost;
			//Marks regions with a social cost value higher than max_ps_cost_ as occupied.
			if (os_cost >= max_os_cost_)
				grid_oS_.data[j * width_ + i] = 100;
			//Otherwise mark them as free
			else
				grid_oS_.data[j * width_ + i] = 0;
			x += resolution_;
		}
		y += resolution_;
	}
}

void SocialGrid::publishOsGrid()
{
	/***
	 * publishOsGrid
	 */
	SocialGrid::computeOsGrid();
	grid_oS_.header.stamp = ros::Time::now();
	grid_oS_Publisher_.publish(grid_oS_);
//    ROS_INFO("Published oS grid");

}

void SocialGrid::computeCompleteGrid()
{
	/***
	 * computeCompleteGrid: PS+OS+map
	 */
	SocialGrid::computePsGrid();
	SocialGrid::computeOsGrid();
	//Copy global map data
	for (uint32_t i = 0; i < global_map_.info.width * global_map_.info.height;
			i++)
	{
		grid_complete_.data[i] = global_map_.data[i];
	}
	int r_map, c_map, map_index_1;
	double value;
	int h = grid_PS_.info.height;
	int w = grid_PS_.info.width;
	//Adding PS grid data
	for (int r = 0; r < h; ++r)
	{

		//ROS_INFO("J=%d", J);
		for (int c = 0; c < w; ++c)
		{
			getMapRCFromGridRC(r, c, r_map, c_map);
			map_index_1 = getIndexFromRC(r_map, c_map, global_map_);
			//ps_index = getIndexFromRC(r,c,grid_PS_);

			//ROS_INFO("I=%d, J=%d", I,J);
			if (c == 0)
			{
				ROS_DEBUG("r=%d, c= %d", r, c);
				ROS_DEBUG("r_map=%d, c_map= %d, map_index_1= %d, ps_index =%d",
						r_map, c_map, map_index_1, (int )(r) * w + c);
			}
			value = grid_PS_.data[(int) (r) * w + c]
					+ grid_oS_.data[(int) (r) * w + c]
					+ global_map_.data[map_index_1];

			grid_complete_.data[map_index_1 - 1] = value;
			grid_complete_.data[map_index_1] = value;
			grid_complete_.data[map_index_1 + 1] = value;

			map_index_1 = getIndexFromRC(r_map - 1, c_map, global_map_);
			grid_complete_.data[map_index_1 - 1] = value;
			grid_complete_.data[map_index_1] = value;
			grid_complete_.data[map_index_1 + 1] = value;

			map_index_1 = getIndexFromRC(r_map + 1, c_map, global_map_);
			grid_complete_.data[map_index_1 - 1] = value;
			grid_complete_.data[map_index_1] = value;
			grid_complete_.data[map_index_1 + 1] = value;



		}
	}
}

void SocialGrid::publishCompleteGrid()
{
	/***
	 * publishCompleteGrid
	 */
	SocialGrid::computeCompleteGrid();
	grid_complete_.info.map_load_time = ros::Time::now();
	grid_complete_.header.stamp = ros::Time::now();
	grid_complete_publisher_.publish(grid_complete_);

}

void SocialGrid::getGridRCFromMapRC(const int rmap, const int cmap, int& rgrid,
		int& cgrid) const
{
	/** Gets the position in the social grid of a point (rmap,cmap) in the global map (grid).
	 *
	 */
	assert(rmap < global_map_.info.height);
	assert(cmap < global_map_.info.width);
	double Oxg = grid_PS_.info.origin.position.x; //origin of PS in x
	double Oyg = grid_PS_.info.origin.position.y;
	double resg = grid_PS_.info.resolution; //resolution of ps grid
	double resm = global_map_.info.resolution; //resolution of global map grid
	double hm = global_map_.info.height; //height of global map
	double hg = grid_PS_.info.height; //height of ps grid

	rgrid = (int) round(hg - (resm / resg) * (hm - rmap) - Oyg / resg);
	cgrid = (int) round((cmap * resm - Oxg) / resg);
}

void SocialGrid::getMapRCFromGridRC(const int rgrid, const int cgrid, int& rmap,
		int& cmap) const
{
	assert(rgrid < grid_PS_.info.height);
	assert(cgrid < grid_PS_.info.width);
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

	double Oxg = (double) grid_PS_.info.origin.position.x; //origin of PS in x
	double Oyg = (double) grid_PS_.info.origin.position.y;
	double resg = (double) grid_PS_.info.resolution; //resolution of ps grid
	double resm = (double) global_map_.info.resolution; //resolution of global map grid
	//double hm = (double) global_map_.info.height; //height of global map
	//double hg = (double) grid_PS_.info.height; //height of ps grid
	//TODO: print this to check why it is always the same value.

	rmap = (int) round((resg * rgrid + Oyg) / resm);
	cmap = (int) round((cgrid * resg + Oxg) / resm);
}

int SocialGrid::getIndexFromRC(int r, int c,
		const nav_msgs::OccupancyGrid & grid) const
{
	assert(r < grid.info.height);
	assert(c < grid.info.width);
	return (r * grid.info.width + c);
}
;

void SocialGrid::getRelXYFromRC(const int r, const int c, double& x_rel,
		double& y_rel, const nav_msgs::OccupancyGrid & grid) const
{
	//Assumes ros convention where the XY origin is in the bottom-left corner and the RC is in the upper-left.
	//std::cout << "r= " << r << ", c= " << c << "  ";
	assert(r < grid.info.height);
	assert(c < grid.info.width);
	double height = (double) grid.info.height;
	x_rel = grid.info.resolution * (c + 0.5);
	y_rel = grid.info.resolution * (height - (r + 0.5));
}

void SocialGrid::getAbsXYFromRC(const int r, const int c, double& x_abs,
		double& y_abs, const nav_msgs::OccupancyGrid & grid) const
{
	//Assumes ros convention where the cell
	double x_relative, y_relative;//x,y coordinates in the grid reference frame.
	//double st, ct;//sin theta, cos theta
	//double h = (double) grid.info.height;
	//std::cout << "abs\n";
	getRelXYFromRC(r, c, x_relative, y_relative, grid);
	//sincos(0 , &st, &ct); //TODO: generalize this to consider angles other than just zero
	//x_abs = x_relative*ct - y_relative*st + grid.info.origin.position.x;
	//y_abs = x_relative*st + y_relative*ct + grid.info.origin.position.y;
	x_abs = x_relative + grid.info.origin.position.x;
	y_abs = y_relative + grid.info.origin.position.y;
}
void SocialGrid::getRCFromRelXY(const double x_rel, const double y_rel, int& r,
		int& c, const nav_msgs::OccupancyGrid & grid) const
{
	double h = (double) grid.info.height;
	c = (int) round((x_rel / grid.info.resolution) - 0.5);
	r = (int) round(h - 0.5 - y_rel / grid.info.resolution);
}
;

void SocialGrid::getRCFromAbsXY(const double x_abs, const double y_abs, int& r,
		int& c, const nav_msgs::OccupancyGrid & grid) const
{

	double x_relative, y_relative;
	//double st, ct;
	//sincos(0 , &st, &ct); //TODO: generalize this to consider angles other than just zero
	x_relative = x_abs - grid.info.origin.position.x;
	y_relative = y_abs - grid.info.origin.position.y;
	getRCFromRelXY(x_relative, y_relative, r, c, grid);
}
;
void SocialGrid::setOrigin()
{
	//Init position according to the first detected person
	x_ = human_poses_.humans[0].x - resolution_ * width_ * 0.5;
	y_ = human_poses_.humans[0].y - resolution_ * height_ * 0.5;
	grid_PS_.info.origin.position.x = x_;
	grid_PS_.info.origin.position.y = y_;
	grid_oS_.info.origin.position.x = x_;
	grid_oS_.info.origin.position.y = y_;
	ROS_INFO("Origin was set at x: %f, y: %f", x_, y_);
}
;
int main(int argc, char **argv)
{

	ros::init(argc, argv, "humanGrids");
	ros::NodeHandle private_nh("~");// Private node handler to read ros parameters
	SocialGrid s_grid;
	//test comment
	double x, y, resolution, time_step;
	int steps, width, height, max_os_cost, max_ps_cost;
	social_filter::humanSocialSpace human_social_space; //!< Sets some properties of the gaussian describing the social spaces
	double social_space_size, social_space_sigma_h, social_space_sigma_r, social_space_sigma_s;
	int social_space_attractiveness;
	//Reading ROS params
	private_nh.param("x", x, 0.0);
	private_nh.param("y", y, 0.0);
	private_nh.param("resolution", resolution, 0.2);
	private_nh.param("steps", steps, 10);
	private_nh.param("time_step", time_step, 0.5);
	private_nh.param("width", width, 35);
	private_nh.param("height", height, 35);
	private_nh.param("max_ps_cost", max_ps_cost, 90);
	private_nh.param("max_os_cost", max_os_cost, 90);
	//Params that modulate social spaces
	private_nh.param("social_space_size", social_space_size,0.0);
	private_nh.param("social_space_sigma_h", social_space_sigma_h,1.0);
	private_nh.param("social_space_sigma_r", social_space_sigma_r,0.5);
	private_nh.param("social_space_sigma_s", social_space_sigma_s,2.0/3.0);
	private_nh.param("social_space_attractiveness", social_space_attractiveness,5);

	human_social_space.size = (float) social_space_size;
	human_social_space.sigma_h = (float) social_space_sigma_h;
	human_social_space.sigma_r = (float) social_space_sigma_r;
	human_social_space.sigma_s = (float) social_space_sigma_s;
	human_social_space.attractiveness = social_space_attractiveness;
	//ROS_INFO("ps_sigma==,%f", ps_sigma);
	//initialize the grid
	s_grid.init(x, y, resolution, steps, time_step, width, height, max_ps_cost,
			max_os_cost, human_social_space);

	ros::Rate r(5);
	while (ros::ok())
	{
		if (s_grid.grid_complete_ready && s_grid.human_poses_ready)
		{
			s_grid.publishPsGrid();
			s_grid.publishCompleteGrid();
		}
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
