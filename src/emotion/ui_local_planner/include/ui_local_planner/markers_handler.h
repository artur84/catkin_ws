#ifndef MARKERS_HANDLER_H_
#define MARKERS_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ui_local_planner/trajectory.h>
#include <ros/ros.h>

namespace ui_local_planner
{

class MarkersHandler
{
public:

	MarkersHandler();
	~MarkersHandler();

	/**
	 * @brief  Init the marker's publisher
	 *  @param frame_id, std::string,  the reference frame of the markers normally '/map' should be ok.
	 */
	virtual void initWriter(std::string & frame_id);

	/**
	 * @brief  publish the current trajectory
	 * @param trajectory, Trajectory, the ui_local_planner trajectory to be displayed in rviz
	 *
	 */
	virtual void write(const std::vector<Trajectory> & trajectories_vector);


private:
	ros::NodeHandle nodeHandle_;
	Trajectory temporal_trajectory_; //Used to allocate temporally each trajectory received in the trajectories_vector
	geometry_msgs::Point temporal_point_;
	ros::Publisher marker_array_pub_; //The ROS publisher of the marker of the trajectory or list of trajectories
	visualization_msgs::Marker points_, line_strip_; //The Markers to plot points of a given trajectory and lines in between points
	visualization_msgs::MarkerArray points_array_; //The Markers to plot points of a given trajectory and lines in between points
	std::vector<double> costs_list_; //To store a list of cost values of the set of trajectories
	double interpolate_( double val, double y0, double x0, double y1, double x1 );
	double base_( double val );
	double red_( double gray );
	double green_( double gray );
	double blue_( double gray );

};


} /* namespace base_local_planner */
#endif /* MARKERS_HANDLER_H_ */
