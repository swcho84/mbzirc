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

// essential header for ROS-OpenCV operation
#include <rclcpp/rclcpp.hpp>

// for using joystick
#include <sensor_msgs/msg/joy.h>

using namespace std;
using namespace rclcpp;

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

#endif  // MBZIRC_JOYSTICK101_GLOBAL_HEADER_H
