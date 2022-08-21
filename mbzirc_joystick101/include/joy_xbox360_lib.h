#ifndef MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H
#define MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace rclcpp;

class JoyXBox360
{
public:
  JoyXBox360(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz);
  ~JoyXBox360();

private:
  std::shared_ptr<rclcpp::Node> parentNode_;
  ConfigParam cfgParam_;

  int nHz_;
};

#endif  // MBZIRC_JOYSTICK101_JOY_XBOX360_LIB_H