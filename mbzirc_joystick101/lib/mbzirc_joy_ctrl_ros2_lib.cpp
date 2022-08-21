#include "mbzirc_joy_ctrl_ros2_lib.h"

using namespace std;
using namespace rclcpp;

MbzircJoyCtrlRos2::MbzircJoyCtrlRos2(const std::string& nodeName, const rclcpp::NodeOptions& options, const int nHz)
  : Node(nodeName, options), nHz_(nHz)
{
  // using other class--->handover the nodeHandle
  nodeHandle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node*) {});

  // setting the configuration class
  cfgParam_ = new ConfigParam(nodeHandle_);
  if (!cfgParam_->GetRosParams())
    RCLCPP_ERROR(this->get_logger(), "Wrong params!! Please check the parameter sheet..");

  // setting the misc function class
  misc_ = new MiscFunc(*cfgParam_);
  joyXbox360_ = new JoyXBox360(nodeHandle_, *cfgParam_, 30);

  // making the main loop
  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)((1 / nHz_) * 1000)),
                                   std::bind(&MbzircJoyCtrlRos2::MainTimerCbLoop, this));
}

MbzircJoyCtrlRos2::~MbzircJoyCtrlRos2()
{
}

// main loop, made by timer
void MbzircJoyCtrlRos2::MainTimerCbLoop()
{
  auto steadyClock = rclcpp::Clock();  // [milliseconds]
  // RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steadyClock, 1000, "[DEBUG]Hello, world! ROS2!");
  // RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steadyClock, 1000, "[WARN]Hello, world! ROS2!");
  // RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), steadyClock, 1000, "[ERROR]Hello, world! ROS2!");
  // RCLCPP_FATAL_STREAM_THROTTLE(this->get_logger(), steadyClock, 1000, "[FATAL]Hello, world! ROS2!");
}
