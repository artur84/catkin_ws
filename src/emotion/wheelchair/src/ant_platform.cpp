/*
 * Proxy class for ANT-powered platform
 * API inspired from BlueBotics Communication Interface in Java
 * and code inspided by Peter Einramhof
 *
 * @author SAR, MYG, LAMG
 *
 * Copyright (C) 2009 INRIA
 */
#include <iostream>
#include <stdio.h>
#include "ant_platform.h"

/**
 * Proxy class for sending commands to an ANT-powered platform.
 *
 * This class delegates its calls to the platform via a LOS connection.
 */
AntPlatform::AntPlatform(std::string host, int port, double timeout)
    : laser_scan_length_(500), odometry_data_length_(9),
    motion_speed_length_(3), is_joy_mode_(false)
{
  // Connect to wheelchair and set stream buffer
  los_.connect(host, port, timeout);

  scan_data_result_.sick_data = (float *)NULL;

  // Check if the size of the buffer for storing SICK laser data is valid.
  if (laser_scan_length_ == 0)
    throw std::runtime_error("AntPlatform::AntPlatform(): Error.LaserScanLength.Zero");

  // Create the buffers
  try
  {
    // Create a buffer for storing SICK laser data.
    scan_data_result_.sick_data = new float[laser_scan_length_ * 2];
    if (scan_data_result_.sick_data == (float *)NULL)
      throw std::runtime_error("cAntPlatform::AntPlatform()::Configuration(): Error.New.Failed");
    scan_data_result_.sick_pose = new double[3];
    scan_data_result_.indices = new int32_t[3];
    scan_data_result_.sick_ages = new int32_t[laser_scan_length_ * 2];
    scan_data_result_.sick_intensities = new int8_t[laser_scan_length_ * 2];

    motion_speed_result_ = new double[motion_speed_length_];
    odometry_data_result_.pose = new double[odometry_data_length_];
    odometry_data_set_.pose = new double[odometry_data_length_];
  }
  catch (std::runtime_error e)
  {
    printf("%s", e.what());
    cleanUp();
    throw;
  }
}

AntPlatform::~AntPlatform()
{
  cleanUp();
}

void AntPlatform::login(std::string user, std::string password) //throw exception
{
  // Prepare the LOS call 'login'...
  los_.init();
  std::string callName = "login";
  los_.writeCall(callName, 2);
  los_.writeString(user);
  los_.writeString(password);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }
  // Check the validity of the expected call result : should be void
  if (los_.readCallResult())
    throw std::runtime_error("AntPlatform::login(): Error.Response.Invalid");
}

void AntPlatform::odometryGetPose() //throw IOException
{
  // Prepare the LOS call 'Odometry.getPose'...
  try
  {
    call("Odometry.getPose", false);
  }
  catch (std::runtime_error &e)
  {
    // Check the validity of the expected call result.
    throw std::runtime_error("AntPlatform::odometryGetPose: Error.Response.Invalid");
  }

  if (los_.readArray() != 2)
    throw std::runtime_error("AntPlatform::odometryGetPose(): Error.Response.Invalid");

  // Extract the data contained in the response.
  odometry_data_result_.time_stamp = (double)los_.readFloat64();
  los_.readFloat64Array(odometry_data_result_.pose, odometry_data_result_.data_count);
  if (odometry_data_result_.data_count != odometry_data_length_)
    throw std::runtime_error("AntPlatform::odometryGetPose: Error.Response.Size");
}

void AntPlatform::odometrySetPose(double x,double y,double theta,double x_var,double y_var,double theta_var,double x_y_cov,double x_theta_cov,double y_theta_cov)
{
 // Prepare the LOS call 'update'...
  los_.init();
  std::string callName = "Odometry.update";
  los_.writeCall(callName, 2);
  los_.writeFloat64(odometry_data_result_.time_stamp); // Will set as timestamp last value got when calling GetPose...
  odometry_data_set_.pose[0]=x;
  odometry_data_set_.pose[1]=y;
  odometry_data_set_.pose[2]=theta;
  odometry_data_set_.pose[3]=x_var;
  odometry_data_set_.pose[4]=y_var;
  odometry_data_set_.pose[5]=theta_var;
  odometry_data_set_.pose[6]=x_y_cov;
  odometry_data_set_.pose[7]=x_theta_cov;
  odometry_data_set_.pose[8]=y_theta_cov;
  
  los_.writeFloat64Array(odometry_data_set_.pose, 9);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }
  // Check the validity of the expected call result : should be void
  if (los_.readCallResult())
    throw std::runtime_error("AntPlatform::Odometry.update(): Error.Response.Invalid");	
}

void AntPlatform::scanGet() //throw IOException
{
  // Prepare the LOS call 'getScanData'...
  los_.init();
  std::string call_name = "Scan.get";
  los_.writeCall(call_name, 1);
  los_.writeInt32(GF_DEFAULT);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();

  // Check if there has been a call exception 
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }


  // All OK, let's check we have a call result.
  if (!los_.readCallResult())
  {
    throw std::runtime_error("AntPlatform::getScanData: Error Result is not a CallResult");
  }

  // We should have an array of 5 elements
  if (los_.readArray() != 5)
  {
    throw std::runtime_error("AntPlatform::getScanData(): Error Response Array argument count != 5");
  }

  // Extract the data contained in the response.

  //First get the time_stamp  
  scan_data_result_.time_stamp = (double)los_.readFloat64();
  
  
  //Then get the pose.
  uint32_t val;
  los_.readFloat64Array(scan_data_result_.sick_pose, val);
  if (val != 3) // Pose should have 3 values
  {
    throw std::runtime_error("AntPlatform::getScanData(): Error Pose Array argument count != 3");
  }

  // Get maxAge of the scan values
  scan_data_result_.max_age =  los_.readInt32();
  
  // Read indices array
  los_.readInt32Array(scan_data_result_.indices, val);
  if (val != 3) // We should have 3 values
  {
    throw std::runtime_error("AntPlatform::getScanData(): Error Indices Array argument count != 3");
  }
      
  // And finally read data from scan
  los_.readFloat32Array(scan_data_result_.sick_data, scan_data_result_.data_count);
  scan_data_result_.data_sick_count = scan_data_result_.indices[0];
  //printf("AntPlatform::getScanData() scan data count = %d %d \n", scan_data_result_.data_sick_count, scan_data_result_.data_count);
  
}


void AntPlatform::getSpeed() //throw IOException
{
  // Prepare the LOS call 'Motion.getSpeed'...
  los_.init();
  std::string call_name = "Motion.getSpeed";
  los_.writeCall(call_name, 1);
  los_.writeInt32(GF_DEFAULT);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();

  // Check if there has been a call exception 
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }


  // All OK, let's check we have a call result.
  if (!los_.readCallResult())
  {
    throw std::runtime_error("AntPlatform::getScanData: Error Result is not a CallResult");
  }

  // We should have an array of 5 elements
//   if (los_.readArray() != 5)
//   {
//     throw std::runtime_error("AntPlatform::getScanData(): Error Response Array argument count != 5");
//   }

  // Extract the data contained in the response.

  //First get the time_stamp  
//   scan_data_result_.time_stamp = (double)los_.readFloat64();
  
  
  //Then get the pose.
  uint32_t val;
  los_.readFloat64Array(motion_speed_result_, val);
  if (val != 3) // Pose should have 3 values
  {
    throw std::runtime_error("AntPlatform::getScanData(): Error Pose Array argument count != 3");
  }

//   // Get maxAge of the scan values
//   scan_data_result_.max_age =  los_.readInt32();
  
//   // Read indices array
//   los_.readInt32Array(scan_data_result_.indices, val);
//   if (val != 3) // We should have 3 values
//   {
//     throw std::runtime_error("AntPlatform::getScanData(): Error Indices Array argument count != 3");
//   }
      
//   // And finally read data from scan
//   los_.readFloat32Array(scan_data_result_.sick_data, scan_data_result_.data_count);
//   scan_data_result_.data_sick_count = scan_data_result_.indices[0];
//   //printf("AntPlatform::getScanData() scan data count = %d %d \n", scan_data_result_.data_sick_count, scan_data_result_.data_count);
  
}

void AntPlatform::motionSetSpeed(double sd, double thetad){
  // Prepare the LOS call 'Motion.setSpeed'...
  los_.init();
  std::string callName = "Motion.setSpeed";
  los_.writeCall(callName, 2);
  los_.writeFloat64(sd);
  los_.writeFloat64(thetad);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }
  // Check the validity of the expected call result : should be void
  if (los_.readCallResult())
    throw std::runtime_error("AntPlatform::SetSpeed(): Error.Response.Invalid");  
}

void AntPlatform::modeSetAutonomous() 
{
  std::string key = "Motion.mode";
  std::string value = "normal";
  
  // Prepare the LOS call 'configure for autonomous mode'...
  los_.init();
  std::string callName = "configure";
  los_.writeCall(callName, 1);
  los_.writeStruct(1);
  los_.writeStructKey(key);
  los_.writeString(value);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }
  
  // Check the validity of the expected call result:
  if (!los_.readCallResult())
    throw std::runtime_error("AntPlatform::Mode_setAutonomous(): Error.Response.Invalid");

  std::string *resStruct = new std::string[1];
  uint32_t arraySize = 0;
  los_.readStringArray(resStruct, arraySize);
  if (arraySize != 0) 
    throw std::runtime_error(resStruct[0].c_str());  
}

void AntPlatform::modeSetJoystick() 
{
  std::string key = "Motion.mode";
  std::string value = "joystick";
  
  // Prepare the LOS call 'configure for manual mode'...
  los_.init();
  std::string callName = "configure";
  los_.writeCall(callName, 1);
  los_.writeStruct(1);
  los_.writeStructKey(key);
  los_.writeString(value);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }
  
  // Check the validity of the expected call result:
  if (!los_.readCallResult())
    throw std::runtime_error("AntPlatform::Mode_setJoystick(): Error.Response.Invalid");

  std::string *resStruct = new std::string[1];
  uint32_t arraySize = 0;
  los_.readStringArray(resStruct, arraySize);
  if (arraySize != 0) 
    throw std::runtime_error(resStruct[0].c_str()); 
  std::cout <<  resStruct[0].c_str() << std::endl;
}
 

double AntPlatform::testNop() //throw IOException
{
  try
  {
    call("Test.nop", false);
  }
  catch (std::runtime_error &e)
  {
    throw e;
  }

  return los_.readFloat64();
}

void AntPlatform::cleanUp()
{
  // Delete the buffer for storing the SICK laser data.
  if (scan_data_result_.sick_data != (float *)NULL)
  {
    delete [] scan_data_result_.sick_pose;
    delete [] scan_data_result_.indices;
    delete [] scan_data_result_.sick_data;
    delete [] scan_data_result_.sick_ages;
    delete [] scan_data_result_.sick_intensities;

    scan_data_result_.sick_pose = (double *)NULL;
    scan_data_result_.indices = (int32_t *)NULL;
    scan_data_result_.sick_data = (float *)NULL;
    scan_data_result_.sick_ages = (int32_t *)NULL;
    scan_data_result_.sick_intensities = (int8_t *)NULL;
  }

  // Delete buffer storing Motion Speed
  if (motion_speed_result_ != (double *)NULL)
  {
    delete [] motion_speed_result_;
    motion_speed_result_ = (double *) NULL;
  }

  // Delete buffer storing Odometry pose
  if (odometry_data_result_.pose != (double *)NULL)
  {
    delete [] odometry_data_result_.pose;
    odometry_data_result_.pose = (double *) NULL;
  }
}

void AntPlatform::call(std::string name, bool is_void)
{
  los_.init();
  los_.writeCall(name, 0);
  // ...and write it to the TCP socket.
  los_.flush();
  // Wait for the response.
  los_.init();
  // Check if there has been a call exception.
  if (los_.getTypeID() == LOS_CALL_EXCEPTION)
  {
    std::string los_call_ex_name, los_call_ex_msg;
    los_.readCallException(los_call_ex_name, los_call_ex_msg);
    throw std::runtime_error(los_call_ex_name + los_call_ex_msg);
  }

  // Check the validity of the expected call result.
  if (is_void)
  {
    if (los_.readCallResult())
      throw std::runtime_error("AntPlatform::call(): Error.Response.Invalid.is_void");
  }
  else
    if (!los_.readCallResult())
      throw std::runtime_error("AntPlatform::call(): Error.Response.Invalid.NOTRESULT");
}
