#ifndef __AR_TARGET_HPP
#define __AR_TARGET_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class ARTarget
{
public:
  ros::Time lastDetectTime;

private:
  cv::KalmanFilter kalman;
  bool isInitialized;

public:
  ARTarget();
  ARTarget(const ARTarget& t);
  
  void init(ros::Time time, double x, double y, double theta);
  
  void predict(ros::Time time);
  void update(ros::Time time, double x, double y, double theta);

  double getX();
  double getY();
  double getTheta();
  double getLinearSpeed();
  double getAngularSpeed();  

};

#endif // __AR_TARGET_HPP
