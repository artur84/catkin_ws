#include <math.h>
#include <tf/tf.h>
#include <social_filter/humanPose.h>
#include <social_filter/humanPoses.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <vector>
#include <math.h>


using namespace std;

class legProc{
public:

  social_filter::humanPoses pos_array;
	ros::NodeHandle n;
  ros::Subscriber pos_sub;
  ros::Publisher pos_pub;
  vector<double> prev_speed;
  vector<double> prev_theta;
  
	legProc();
	~legProc();
  void posCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg);
	void pub();
  
};
