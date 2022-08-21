#include "joy_xbox360_lib.h"

using namespace std;
using namespace rclcpp;

JoyXBox360::JoyXBox360(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz)
  : parentNode_(parentNode), cfgParam_(cfg), nHz_(nHz)
{
  // for debugging
  printf("JoyXbox360::fVelHeave:%.4f\n", cfgParam_.joyCfgXbox.ctrlScale.fVelHeave);
  printf("JoyXbox360::JoyNodeTopic::%s\n", cfgParam_.strJoyTpNm.c_str());
}

JoyXBox360::~JoyXBox360()
{
}
