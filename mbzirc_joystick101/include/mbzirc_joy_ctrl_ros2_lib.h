#ifndef MBZIRC_JOYSTICK101_MBZIRC_JOY_CTRL_ROS2_LIB_H
#define MBZIRC_JOYSTICK101_MBZIRC_JOY_CTRL_ROS2_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace rclcpp;

class MbzircJoyCtrlRos2 : public Node
{
public:
  MbzircJoyCtrlRos2(const std::string& nodeName, const rclcpp::NodeOptions& options, int nHz);
  ~MbzircJoyCtrlRos2();

private:
  ConfigParam* cfg_;

  std::shared_ptr<rclcpp::Node> nodeHandle_;
  TimerBase::SharedPtr timer_;

  void MainTimerCbLoop();
};

#endif