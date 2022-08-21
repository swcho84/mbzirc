#ifndef MBZIRC_JOYSTICK101_CONFIG_PARAM_H
#define MBZIRC_JOYSTICK101_CONFIG_PARAM_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace rclcpp;

class RosParamNotFoundException : public std::exception
{
public:
  string key;
  explicit RosParamNotFoundException(const string& key_)
  {
    key = key_;
  }
  virtual const char* what() const throw()
  {
    string msg = "Failed to read param at key ";
    return (msg + key).c_str();
  }
};

class ConfigParam
{
public:
  ConfigParam(const std::shared_ptr<rclcpp::Node>& parentNode);
  ~ConfigParam();

  bool GetRosParams();	

	JoyRaw joyCfgXbox;

	string strJoyTpNm;

	float fAttPsi;

private:
  std::shared_ptr<rclcpp::Node> parentNode_;

	CtrlRefScale joyMoveScale_;

	string strHomeName_;

	bool ReadRosParams();
  void ReadRosParam(const string& key, float& val);
  void ReadRosParam(const string& key, double& val);
  void ReadRosParam(const string& key, bool& val);
  void ReadRosParam(const string& key, int32_t& val);
  void ReadRosParam(const string& key, string& val);

};

#endif