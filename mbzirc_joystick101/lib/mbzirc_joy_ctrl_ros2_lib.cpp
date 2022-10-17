#include "mbzirc_joy_ctrl_ros2_lib.h"

using namespace std;
using namespace rclcpp;

MbzircJoyCtrlRos2::MbzircJoyCtrlRos2(const std::string& nodeName, const rclcpp::NodeOptions& options)
  : Node(nodeName, options)
{
  // using other class--->handover the nodeHandle
  nodeHandle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node*) {});

  // setting the configuration class
  cfgParam_ = new ConfigParam(nodeHandle_);
  if (!cfgParam_->GetRosParams())
    RCLCPP_ERROR(this->get_logger(), "Wrong params!! Please check the parameter sheet..");

  // setting the misc function class
  misc_ = new MiscFunc(*cfgParam_);
  joyXbox360_ = new JoyXBox360(nodeHandle_, *cfgParam_, cfgParam_->nJoyNodeHz);
  mbzQuad1ImuData_ = new MbzQuadImuData(nodeHandle_, *cfgParam_, cfgParam_->nQuadImuNodeHz);
  currJoyMbzDroneCtrlCmd_ = std::make_shared<JoyCtrlCmd>();

  // making the main loop
  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)((1 / 1.9) * cfgParam_->nMainLoopHz)),
                                   std::bind(&MbzircJoyCtrlRos2::MainTimerCbLoop, this));
}

MbzircJoyCtrlRos2::~MbzircJoyCtrlRos2()
{
}

// main loop, made by timer
void MbzircJoyCtrlRos2::MainTimerCbLoop()
{
  currJoyMbzDroneCtrlCmd_ = (joyXbox360_->GenJoyInfoLoop());

  // for debugging
  // RCLCPP_INFO(this->get_logger(), "rpy:(%.4lf,%.4lf,%.4lf)", currJoyMbzDroneCtrlCmd_->ctrlMove.fRoll,
  //             currJoyMbzDroneCtrlCmd_->ctrlMove.fPitch, currJoyMbzDroneCtrlCmd_->ctrlMove.fYaw);
  // RCLCPP_INFO(this->get_logger(), "thr:(%.4lf)", currJoyMbzDroneCtrlCmd_->ctrlMove.fThr);
  // RCLCPP_INFO(this->get_logger(), "rp_accum:(%.4lf,%.4lf)", currJoyMbzDroneCtrlCmd_->ctrlAuxMove.fRoll,
  //             currJoyMbzDroneCtrlCmd_->ctrlAuxMove.fPitch);
  // RCLCPP_INFO(this->get_logger(), "autojoy:(%d,%d)", (int)(currJoyMbzDroneCtrlCmd_->bCtrlAutoMode),
  //             (int)(currJoyMbzDroneCtrlCmd_->bCtrlJoyMode));
  // RCLCPP_INFO(this->get_logger(), "rd_gear:(%d,%d)", currJoyMbzDroneCtrlCmd_->nRgearStatus,
  //             currJoyMbzDroneCtrlCmd_->nDgearStatus);
  // RCLCPP_INFO(this->get_logger(), " ");
  // RCLCPP_INFO(this->get_logger(), " ");
  // RCLCPP_INFO(this->get_logger(), " ");
}
