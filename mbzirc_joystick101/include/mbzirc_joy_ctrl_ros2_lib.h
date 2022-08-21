#ifndef MBZIRC_JOYSTICK101_MBZIRC_JOY_CTRL_ROS2_LIB_H
#define MBZIRC_JOYSTICK101_MBZIRC_JOY_CTRL_ROS2_LIB_H

#include "global_header.h"
#include "joy_xbox360_lib.h"
#include "config_param.h"
#include "misc_func.h"

using namespace std;
using namespace rclcpp;

class MbzircJoyCtrlRos2 : public Node
{
public:
  MbzircJoyCtrlRos2(const std::string& nodeName, const rclcpp::NodeOptions& options);
  ~MbzircJoyCtrlRos2();

private:
  std::shared_ptr<rclcpp::Node> nodeHandle_;
  ConfigParam* cfgParam_;
  MiscFunc* misc_;

  JoyXBox360* joyXbox360_;

  std::shared_ptr<JoyCtrlCmd> joyMbzDroneCtrlCmd_;

  TimerBase::SharedPtr timer_;

  void MainTimerCbLoop();
};

#endif