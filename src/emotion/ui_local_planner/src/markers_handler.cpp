#include <ui_local_planner/markers_handler.h>

namespace ui_local_planner
{

MarkersHandler::MarkersHandler()
{
}

MarkersHandler::~MarkersHandler()
{
}

void MarkersHandler::initWriter(std::string & frame_id)
{
	marker_array_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(
			"trajectory_marker_array", 1);
	points_.header.frame_id = line_strip_.header.frame_id = frame_id;
	points_.ns = line_strip_.ns = "ui_trajectories"; //The use of this  name_space ensures the markers will not collide with other broadcasters
	points_.action = line_strip_.action = visualization_msgs::Marker::ADD;
	points_.pose.orientation.w = line_strip_.pose.orientation.w = 1.0;
	line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
	//changing the size of the markers
	line_strip_.scale.x = 0.05;
	//POINTS ATTRIBUTES
	points_.type = visualization_msgs::Marker::POINTS;
	points_.scale.x = points_.scale.y = 0.02;

}

void MarkersHandler::write(const std::vector<Trajectory> & trajectories_vector)
{


	if (trajectories_vector.size() <= 0)
	{
		ROS_WARN("The trajectories vector is empty, will not print any marker");
	}
	else
	{
		points_.header.stamp = line_strip_.header.stamp = ros::Time::now();
		points_.lifetime = line_strip_.lifetime = ros::Duration(0.5);
		double theta, factor = 1.0; //factor is used to normalize cost values between 0 an 1.
		double gray;
		//Erase the marker printed in last iteration
		points_array_.markers.clear();

		//Getting the maximum of trajectory costs
		costs_list_.clear();
		for (uint32_t n = 0; n < trajectories_vector.size(); n++)
		{
			costs_list_.push_back(trajectories_vector[n].cost_);
		}
		factor = (double) (*std::max_element(costs_list_.begin(), costs_list_.end())+0.001);
		//ROS_INFO("trajectory max cost= %f", factor);
		// Create the vertices for the points and lines from trajectory
		for (uint32_t n = 0; n < trajectories_vector.size(); n++)
		{
			temporal_trajectory_ = trajectories_vector[n];
			points_.id = n;
			line_strip_.id = 1000 + n; //unless we have more than 1000 trajectories it will work ok
			//color

			if (temporal_trajectory_.cost_ == -1.0) //print it in gray to show not valid path
			{
	//			points_.color.g = 0.6;
	//			points_.color.r = 0.6;
	//			points_.color.b = 0.6;
	//			points_.color.a = 0.3;
				//line strip is red
				line_strip_.color.r = 0.0;
				line_strip_.color.g = 0.0;
				line_strip_.color.b = 0.0;
				line_strip_.color.a = 0.7;
			}
			else  //valid paths are printed with a color according to it's cost
			{
				gray=1.0 - temporal_trajectory_.cost_ / factor;
			line_strip_.color.g = red_(gray);
			line_strip_.color.r = green_(gray);
			line_strip_.color.b = blue_(gray);
				line_strip_.color.a = 1.0;
			//line_strip_.scale.x = 0.02 * gray;
				//color of the dots
			points_.color.g = red_(gray);
			points_.color.r = green_(gray);
			points_.color.b = blue_(gray);
				points_.color.a = 1.0;
			//points_.scale.x = points_.scale.y = 0.03 * gray;

			}
			//temporal_point_.z = 0.5 * temporal_trajectory_.cost_;
			//Init a new set of points for the current trajectory
			points_.points.clear();
			line_strip_.points.clear();
			//if the trajectory has not elements don't print it
			//if (temporal_trajectory_.getPointsSize() >= 1)
			//{
				//print each point or line in the trajectory
				for (uint32_t i = 0; i < temporal_trajectory_.getPointsSize(); ++i)
				{

					//gets the point in the ith position of the trajectory
					temporal_trajectory_.getPoint(i, temporal_point_.x,
							temporal_point_.y, theta);
					//push the point into the marker's lists
					points_.points.push_back(temporal_point_);
					line_strip_.points.push_back(temporal_point_);
				}
				points_array_.markers.push_back(points_);
				points_array_.markers.push_back(line_strip_);
			//}
			//else
				//ROS_WARN("The trajectory should have at least two elements, it will not be printed");
		}
		//Only if we have at least one valid marker in the array it will be printed
		//if (points_array_.markers.size()>0)
			//publish the markers
			marker_array_pub_.publish(points_array_);
	}

}


double MarkersHandler::interpolate_( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double MarkersHandler::base_( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate_( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate_( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

double MarkersHandler::red_( double gray ) {
    return base_( gray - 0.5 );
}
double MarkersHandler::green_( double gray ) {
    return base_( gray );
}
double MarkersHandler::blue_( double gray ) {
    return base_( gray + 0.5 );
}
} /* namespace ui_local_planner */
