#ifndef MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H
#define MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H

#include "global_header.h"
#include "config_param.h"
#include "misc_func.h"

using namespace std;
using namespace rclcpp;

class JoyXBox360
{
public:
  JoyXBox360(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz);
  ~JoyXBox360();

  std::shared_ptr<JoyCtrlCmd> GenJoyInfoLoop();

private:
  std::shared_ptr<rclcpp::Node> parentNode_;
  ConfigParam cfgParam_;
  MiscFunc misc_;

  Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoyRaw_;
  void CbJoyRaw(const sensor_msgs::msg::Joy::SharedPtr msgRaw);

  JoyCtrlCmd GenJoyCtrlAxisRefInfo(JoyRaw param, const sensor_msgs::msg::Joy::SharedPtr joyRaw, JoyCtrlCmd res);
  JoyCtrlCmd GenJoyCtrlBtnsRefInfo(JoyRaw param, const sensor_msgs::msg::Joy::SharedPtr joyRaw, JoyCtrlCmd res);

  JoyCtrlCmd joyCtrlRef_;

  int nHz_;

  bool bPrevUseJoyConLoop_;
  bool bPrevUseExtGuidLoop_;
};

#endif  // MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H