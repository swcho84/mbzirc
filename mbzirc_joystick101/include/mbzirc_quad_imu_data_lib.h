#ifndef MBZIRC_JOYSTICK101_MBZIRC_QUAD_IMU_DATA_LIB_H
#define MBZIRC_JOYSTICK101_MBZIRC_QUAD_IMU_DATA_LIB_H

#include "global_header.h"
#include "config_param.h"
#include "misc_func.h"

using namespace std;
using namespace rclcpp;

class MbzQuadImuData
{
public:
  MbzQuadImuData(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz);
  ~MbzQuadImuData();

  std::shared_ptr<EulerImuData> GenImuInfoLoop();

private:
  std::shared_ptr<rclcpp::Node> parentNode_;
  ConfigParam cfgParam_;
  MiscFunc misc_;

  EulerImuData currEulerImuData_;

  Subscription<sensor_msgs::msg::Imu>::SharedPtr subQuadImuRaw_;
  void CbQuadImuRaw(const sensor_msgs::msg::Imu::SharedPtr msgRaw);

  int nHz_;
};

#endif  // MBZIRC_JOYSTICK101_MBZIRC_QUAD_IMU_DATA_LIB_H