/*
 * Proxy class for ANT-powered platform
 * Code inspired from BlueBotics Communication Interface in Java
 *
 * @author SAR, MYG LAMG
 *
 * Copyright (C) 2009 INRIA
 */

#ifndef _ANTPLATFORM_H_
#define _ANTPLATFORM_H_

#include <string>
#include <vector>
#include "cpp_los/cpp_los.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Data structure that stores the result of the platform command
// 'getScanData'. It comprises a time_stamp, the platform pose (Xpos,
// Ypos and Orientation) and the (SICK) laser data. The laser data
// consists of 'LaserScanLength' 2D-points (in platform coordiantes).
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
typedef struct
{
  double time_stamp;
  int8_t joy_x;
  int8_t joy_y;
} JoystickStr;

typedef struct
{
  double time_stamp;
  double *sick_pose;
  int32_t max_age;
  int32_t *indices;
  uint32_t data_count;
  uint32_t data_sick_count;
  float *sick_data;
  int32_t *sick_ages;
  int8_t *sick_intensities;
} ScanDataStr;

typedef struct
{
  double time_stamp; /**< time of the request as absolute UTC time */
  std::string state; /**< state of the current motion operation  like Ready */
  std::string result; /**< result of the last terminated operation  like Sucess */
} MotionStatusStr;

typedef struct
{
  double time_stamp;
  double *pose;
  uint32_t data_count;
} OdometryDataStr;

typedef struct
{
    double linear_speed;
    double angular_speed;
} SpeedStr;

/**
 * Proxy class for sending commands to an ANT-powered platform.
 *
 * This class delegates its calls to the platform via a LOS connection.
 */
class AntPlatform
{
  private:
    CppLos los_;
    //
    uint32_t laser_scan_length_;
    uint32_t odometry_data_length_;
    uint32_t motion_speed_length_;
    //
    bool is_joy_mode_;

    void cleanUp();

    void call(std::string name, bool is_void);

  public:
    JoystickStr joystick_result_;
    ScanDataStr scan_data_result_;
    OdometryDataStr odometry_data_result_;
    OdometryDataStr odometry_data_set_;
    MotionStatusStr motion_status_result_;
    double *motion_speed_result_;

    /**
    * Create a proxy object.
    *
    * @param host the hostname or IP address of the server
    * @param port the port to connect to
    * @param timeout socket communication timeout (us)
    */
    AntPlatform(std::string host, int port, double timeout);
    ~AntPlatform();

    /**
    * Authenticate the current connection.
    *
    * @param user the user to authenticate as
    * @param password the password for the given user
    */
    void login(std::string user, std::string password);// throws IOException

    /**
    * Get the current pose of the platform.
    *
    * Set pose a 2-element tuple containing the current time [s] and the
    * pose and its associated covariances. pose is allocated after call
    */
    void odometryGetPose();// throws IOException
    
    /**
    * Set the current pose of the platform.
    *
    */
    void odometrySetPose(double x,double y,double theta,double x_var,double y_var,double theta_var,double x_y_cov,double x_theta_cov,double y_theta_cov);// throws IOException

    /**
    * Get the pose of the platform at the given time.
    * Set pose a 2-element tuple containing the time of the pose [s], and the
    *     pose and its associated covariances. pose allocated after call.
    *
    * @param time the time for which the pose is desired [s]
    */
    void odometryGetPose(double time); // throws IOException

    /*
    * Scan point types.
    */
    static const int PT_SYNCHRONOUS = 0;
    static const int PT_EXTERNAL = 1;
    static const int PT_VIRTUAL = 2;
    static const int PT_ULTRASOUND = 3;
    static const int PT_INFRARED = 4;

    /*
    * Scan retrieval flags.
    */
    static const int GF_COORDINATES = 1 << 0;
    static const int GF_COORDINATES_3D = 1 << 1;
    static const int GF_AGES = 1 << 2;
    static const int GF_INTENSITIES = 1 << 3;
    static const int GF_TYPES = 1 << 4;
    static const int GF_SYNC = 1 << 8;
    static const int GF_SYNC_MEM = 1 << 9;
    static const int GF_ASYNC = 1 << 10;
    static const int GF_ASYNC_MEM = 1 << 11;    
    static const int GF_ALL_POINTS = GF_SYNC | GF_SYNC_MEM | GF_ASYNC | GF_ASYNC_MEM;
    //static const int GF_DEFAULT = GF_COORDINATES | GF_AGES | GF_INTENSITIES | GF_SYNC;//GF_ALL_POINTS;
    static const int GF_DEFAULT = GF_COORDINATES | GF_SYNC;//GF_ALL_POINTS;
    static const int GF_DEFAULT_3D = GF_COORDINATES_3D | GF_AGES | GF_INTENSITIES | GF_ALL_POINTS;

    /**
    * Get scan data : using scan configuration by default as
    * GF_COORDINATES | GF_AGES | GF_INTENSITIES | GF_ALL_POINTS
    * Set scan_data_result_ attribute
    */
    void scanGet(); // throws IOException
    void getSpeed(); // throws IOException

  public:
    /**
    * Do nothing and return PI.
    *
    * @return the number PI
    */
    double testNop(); // throws IOException
    
    /**
    * Set plattform translation and rotation speed target.
    *
    * @param sd The desired platform translation speed [m/s]
    * @param thetad The desired platform rotation speed [rad/s]
    *   
    */
    void motionSetSpeed(double sd, double thetad);
    
    /**
    * Get plattform translation and rotation speed target.
    *   
    */
    void motionGetSpeed();
    
    /**
    * Set plattform in autonomous mode.
    */
    void modeSetAutonomous();
    
    /**
    * Set plattform in joystick mode.
    */
    void modeSetJoystick();
    
    

};
#endif
