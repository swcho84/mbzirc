#ifndef MBZIRC_JOYSTICK101_GLOBAL_HEADER_H
#define MBZIRC_JOYSTICK101_GLOBAL_HEADER_H

// using vector type data
#include <iostream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <ctime>
#include <vector>
#include <dirent.h>
#include <fstream>
#include <random>
#include <chrono>
#include <cstdio>

// for using boost asio library
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/filesystem.hpp>

// for using eigen library
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// for using tf w.r.t the quaternion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// essential header for ROS-OpenCV operation
#include <rclcpp/rclcpp.hpp>

// for using joystick
#include "sensor_msgs/msg/joy.hpp"

// for using imu data
#include "sensor_msgs/msg/imu.hpp"

using namespace std;
using namespace rclcpp;
using namespace Eigen;

#define PI 3.141592
#define R2D 180.0 / PI
#define D2R PI / 180.0

typedef struct
{
  int nRollChannel;
  int nPitchChannel;
  int nYawChannel;
  int nThrChannel;
  float fRoll;
  float fPitch;
  float fYaw;
  float fThr;
  float fRollAxis;
  float fPitchAxis;
  float fYawAxis;
  float fThrAxis;
} CtrlAxis;

typedef struct
{
  int nRollChannel;
  int nPitchChannel;
  int nYawLtChannel;
  int nYawRtChannel;
  float fRoll;
  float fPitch;
  float fYawLt;
  float fYawRt;
  float fRollAxis;
  float fPitchAxis;
  float fYawLtAxis;
  float fYawRtAxis;
} CtrlAuxilAxis;

typedef struct
{
  int nTrigJoyCtrl;
  int nTrigAutoCtrl;
  int nTrigRgear;
  int nTrigDgear;
} CtrlButton;

typedef struct
{
  float fAttPhi;
  float fAttTheta;
  float fAttRatePsi;
  float fVelHeave;
  float fPhiAuxAccumScale;
  float fThetaAuxAccumScale;
  float fPhiAuxAccumMax;
  float fThetaAuxAccumMax;
  float fPsiAuxMin;
  float fPsiAuxMax;
  int nPsiResol;
  float fThrAuxMin;
  float fThrAuxMax;
  int nThrResol;
} CtrlRefScale;

typedef struct
{
  CtrlAxis ctrlMove;
  CtrlAuxilAxis ctrlAuxMove;
  CtrlButton ctrlButton;
  CtrlRefScale ctrlScale;
} JoyRaw;

typedef struct
{
  CtrlAxis ctrlMove;
  CtrlAuxilAxis ctrlAuxMove;
  CtrlRefScale ctrlScale;
  bool bCtrlAutoMode;
  bool bCtrlJoyMode;
  int nRgearStatus;
  int nDgearStatus;
} JoyCtrlCmd;

typedef struct
{
  Quaterniond quat;
  Vector3d acc;
  Vector3d eulerAng;
  Vector3d eulerRate;
} EulerImuData;

#endif  // MBZIRC_JOYSTICK101_GLOBAL_HEADER_H
