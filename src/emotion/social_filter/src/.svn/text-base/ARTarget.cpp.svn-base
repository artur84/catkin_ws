#include "ARTarget.hpp"

#include <cmath>

ARTarget::ARTarget()
  : kalman(8,4,0,CV_64F)
  , isInitialized(false)
{
}

ARTarget::ARTarget(const ARTarget& t)
  : lastDetectTime(t.lastDetectTime)
  , kalman(t.kalman)
  , isInitialized(t.isInitialized)
{
  cv::setIdentity(kalman.transitionMatrix);
  cv::setIdentity(kalman.measurementMatrix);
  cv::setIdentity(kalman.processNoiseCov);
  cv::setIdentity(kalman.measurementNoiseCov);
    
//   kalman.transitionMatrix.at<double>(1,1) = 1;
//   kalman.transitionMatrix.at<double>(2,2) = 1;
//   kalman.transitionMatrix.at<double>(3,3) = 1;
//   kalman.transitionMatrix.at<double>(4,4) = 1;
//   kalman.transitionMatrix.at<double>(5,5) = 1;
//   kalman.transitionMatrix.at<double>(2,2) = 1;
//   kalman.transitionMatrix.at<double>(3,3) = 1;
  
}  

void ARTarget::init(ros::Time time, double x, double y, double theta)
{
  lastDetectTime = time;

  kalman.statePost.at<double>(0) = x;
  kalman.statePost.at<double>(1) = y;
  kalman.statePost.at<double>(2) = cos(theta);
  kalman.statePost.at<double>(3) = sin(theta);
  kalman.statePost.at<double>(4) = 0;
  kalman.statePost.at<double>(5) = 0;
  kalman.statePost.at<double>(6) = 0;
  kalman.statePost.at<double>(7) = 0;

  kalman.statePre.at<double>(0) = x;
  kalman.statePre.at<double>(1) = y;
  kalman.statePre.at<double>(2) = cos(theta);
  kalman.statePre.at<double>(3) = sin(theta);
  kalman.statePre.at<double>(4) = 0;
  kalman.statePre.at<double>(5) = 0;
  kalman.statePre.at<double>(6) = 0;
  kalman.statePre.at<double>(7) = 0;  
}

void ARTarget::predict(ros::Time time)
{
  ros::Duration d = time - lastDetectTime;
  double dt = d.toSec();
  kalman.transitionMatrix.at<double>(0,4) = dt;
  kalman.transitionMatrix.at<double>(1,5) = dt;
  kalman.transitionMatrix.at<double>(2,6) = dt;
  kalman.transitionMatrix.at<double>(3,7) = dt;
  
  kalman.predict();
}

void ARTarget::update(ros::Time time, double x, double y, double theta)
{
  //cv::Vec4d measurement(x, y, cos(theta), sin(theta));
  //cv::Mat measurement = cv::Mat_<double>(4, 1);
  double data[4] = {x, y, cos(theta), sin(theta)};

  cv::Mat measurement(4,1,CV_64F, data);
  kalman.correct(measurement);
  //kalman.correct((const cv::Mat&) measurement);
}

double ARTarget::getX()
{
  return kalman.statePost.at<double>(0);
}

double ARTarget::getY()
{
  return kalman.statePost.at<double>(1);
}

double ARTarget::getTheta()
{
  return atan2(kalman.statePost.at<double>(3), kalman.statePost.at<double>(0, 2));
}

double ARTarget::getLinearSpeed()
{
  double vx = kalman.statePost.at<double>(4);
  double vy = kalman.statePost.at<double>(5);
  return sqrt(vx*vx + vy*vy);
}

double ARTarget::getAngularSpeed()
{
  return 0;
}
