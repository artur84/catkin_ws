#ifndef SOCIAL_FCNS_H
#define SOCIAL_FCNS_H

#include "social_filter/int_data.h"
#include "social_filter/int_list.h"
#include "tf/transform_listener.h"
#include "social_filter/humanPose.h"

//The value could be changed related to the space we want to model
//.41  makes that personal space in the front gets 1.2m + .5m of inflation
//inflation must take into account size of robot 
#define PSPACE_LIMIT 0.41  // 0.45 intimate space-- 1.2 Personal space
#define GRAD2RADIAN .01745
#define NONE 0
#define VISVIS 1
#define V_FORM 2
#define L_FORM 3
#define C_FORM 4
#define CIRCULAR 5
#define HOI 6

#define MAX_P 15


//The angles are in radians
struct dataAlign{
  double _alfa;
  double _beta;
  int side;
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
int  EvFormation(double  x1,double y1,double theta1,double vx1,double vy1,double  x2,double y2,double theta2,double vx2,double vy2, social_filter::int_data* Data, double interaction_space);
int  EvObjInt(double  x1,double y1,double theta1,double vx1,double vy1,double  x2,double y2,double theta2,social_filter::int_data* Data);
double PSpace(double xk, double yk, double xp, double yp, double dir, double pspace_size);
void Alignment(double x1,double y1,double theta1, double x2,double  y2,double theta2, struct dataAlign* Data); 
double EvalGauss(double x, double y,const social_filter::int_data Data );
void transformPosetoMap(tf::TransformListener* listener,social_filter::humanPose on_odom, social_filter::humanPose* on_map);
int elements(social_filter::int_data a,social_filter::int_data b,social_filter::int_data* c);
int min_edges(int k);
int n_edges(social_filter::int_data a, int form_matrix[MAX_P][MAX_P]);
bool group_tightness(social_filter::int_data a, social_filter::int_data b,int M[MAX_P][MAX_P]);
double ips(double x, double y,double theta, double x_hi,double y_hi,double theta_hi,double v_hi);

#endif
