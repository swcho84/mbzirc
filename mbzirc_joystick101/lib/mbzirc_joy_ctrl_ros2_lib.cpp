#include "mbzirc_joy_ctrl_ros2_lib.h"

using namespace std;
using namespace rclcpp;

MbzircJoyCtrlRos2::MbzircJoyCtrlRos2(const std::string& nodeName, const rclcpp::NodeOptions& options, int nHz)
  : Node(nodeName, options)
{
  // setting the nodeHandle
  nodeHandle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node*) {});

  // using other class--->handover the nodeHandle
  // setting the configuration class
  cfg_ = new ConfigParam(nodeHandle_);
  if (!cfg_->GetRosParams())
    RCLCPP_ERROR(this->get_logger(), "Wrong params!! Please check the parameter sheet..");

  // making the main loop
  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)((1 / nHz) * 1000)),
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
