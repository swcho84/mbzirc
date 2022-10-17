#include "mbzirc_quad_imu_data_lib.h"

using namespace std;
using namespace rclcpp;

MbzQuadImuData::MbzQuadImuData(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz)
  : parentNode_(parentNode), cfgParam_(cfg), misc_(cfg), nHz_(nHz)
{
  // generating subscriber, imu node
  RCLCPP_INFO(parentNode_->get_logger(), "Subscriber::IMU::%s", cfgParam_.strQuadImuTpNm.c_str());
  auto qosImuRaw = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  subQuadImuRaw_ = parentNode_->create_subscription<sensor_msgs::msg::Imu>(
      cfgParam_.strQuadImuTpNm, qosImuRaw, std::bind(&MbzQuadImuData::CbQuadImuRaw, this, std::placeholders::_1));
}

MbzQuadImuData::~MbzQuadImuData()
{
}

// generating the imu data, quad1, global
std::shared_ptr<EulerImuData> MbzQuadImuData::GenImuInfoLoop()
{
  return std::make_shared<EulerImuData>(currEulerImuData_);
}

// callback function, raw joystick input
void MbzQuadImuData::CbQuadImuRaw(const sensor_msgs::msg::Imu::SharedPtr msgRaw)
{
  // for debugging
  RCLCPP_INFO(parentNode_->get_logger(), "quat:(x,y,z,w):(%.4lf,%.4lf,%.4lf,%.4lf)", msgRaw->orientation.x,
              msgRaw->orientation.y, msgRaw->orientation.z, msgRaw->orientation.w);
}