#ifndef SOCIAL_FCNS_H
#define SOCIAL_FCNS_H

#include "social_filter/int_data.h"
#include "social_filter/int_list.h"
#include "tf/transform_listener.h"
#include "social_filter/humanPose.h"
#include "social_filter/humanSocialSpace.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

//The value could be changed related to the space we want to model
//.41  makes that personal space in the front gets 1.2m + .5m of inflation
//inflation must take into account size of robot
#define PSPACE_LIMIT 0.41  // 0.45 intimate space-- 1.2 Personal space
#define DEG2RAD .01745
#define RAD2DEG  57.2957

#define NONE 0
#define VISVIS 1
#define V_FORM 2
#define L_FORM 3
#define C_FORM 4
#define CIRCULAR 5
#define HOI 6
#define I_FORM 7

#define MAX_P 15

//The angles are in radians
struct dataAlign
{
	double _alfa; //?
	double _beta; //?
	double gamma; //[rad] angle between the visual axis of the two persons
};

/*
 struct dataGaussianBi{
 double media_x;
 double media_y;
 double sd_x;
 double sd_y;
 double angle;//Is in radians
 };
 */
int dummy_EvFormation(double x1, double y1, double x2, double y2,
    social_filter::int_data* Data, double interaction_space);
int EvFormation(double x1, double y1, double phi1, double vx1, double vy1,
		double x2, double y2, double phi2, double vx2, double vy2,
		social_filter::int_data* Data, double interaction_space);
int EvObjInt(double x1, double y1, double phi1, double vx1, double vy1,
		double x2, double y2, double phi2, social_filter::int_data* Data);
double PSpace(double xk, double yk, double xp, double yp, double dir,
		social_filter::humanSocialSpace hss);
bool compute_vortex(double x1, double y1, double phi1, double x2, double y2,
		double phi2, std::vector<double> & vi);
double compute_side_side_distance(double x1, double y1, double x2, double y2,
		double phi2);
void Alignment(double x1, double y1, double phi1, double x2, double y2,
		double phi2, struct dataAlign* Data);
double EvalGauss(double x, double y, const social_filter::int_data Data);
void transformPosetoMap(tf::TransformListener* listener,
		social_filter::humanPose on_odom, social_filter::humanPose* on_map);
int elements(social_filter::int_data a, social_filter::int_data b,
		social_filter::int_data* c);
int min_edges(int k);
int n_edges(social_filter::int_data a, int form_matrix[MAX_P][MAX_P]);
bool group_tightness(social_filter::int_data a, social_filter::int_data b,
		int M[MAX_P][MAX_P]);
double ips(double x, double y, double theta, double x_hi, double y_hi,
		double theta_hi, double v_hi);

#endif
