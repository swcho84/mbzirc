#include "joy_xbox360_lib.h"

using namespace std;
using namespace rclcpp;

JoyXBox360::JoyXBox360(const std::shared_ptr<rclcpp::Node>& parentNode, const ConfigParam& cfg, const int nHz)
  : parentNode_(parentNode), cfgParam_(cfg), misc_(cfg), nHz_(nHz)
{
  // generating subscriber, joy node
  RCLCPP_INFO(parentNode_->get_logger(), "Subscriber::JOY::%s", cfgParam_.strJoyTpNm.c_str());
  auto qosJoyRaw = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  subJoyRaw_ = parentNode_->create_subscription<sensor_msgs::msg::Joy>(
      cfgParam_.strJoyTpNm, qosJoyRaw, std::bind(&JoyXBox360::CbJoyRaw, this, std::placeholders::_1));

  // setting the initialized value
  currJoyCtrlRef_.ctrlAuxMove.fRoll = 0.0f;
  currJoyCtrlRef_.ctrlAuxMove.fPitch = 0.0f;
  currJoyCtrlRef_.ctrlAuxMove.fYawLt = 0.0f;
  currJoyCtrlRef_.ctrlAuxMove.fYawRt = 0.0f;
  bPrevUseExtGuidLoop_ = false;
  bPrevUseJoyConLoop_ = true;
}

JoyXBox360::~JoyXBox360()
{
}

// generating the joystick control reference information, global
std::shared_ptr<JoyCtrlCmd> JoyXBox360::GenJoyInfoLoop()
{
  return std::make_shared<JoyCtrlCmd>(currJoyCtrlRef_);
}

// callback function, raw joystick input
void JoyXBox360::CbJoyRaw(const sensor_msgs::msg::Joy::SharedPtr msgRaw)
{
  // generating the joystick control reference information, axes and buttons
  currJoyCtrlRef_ = GenJoyCtrlAxisRefInfo(cfgParam_.joyCfgXbox, msgRaw, currJoyCtrlRef_);
  currJoyCtrlRef_ = GenJoyCtrlBtnsRefInfo(cfgParam_.joyCfgXbox, msgRaw, currJoyCtrlRef_);

  // for debugging
  // RCLCPP_INFO(parentNode_->get_logger(), "rpy:(%.4lf,%.4lf,%.4lf)", joyCtrlRef_.ctrlMove.fRoll,
  //             joyCtrlRef_.ctrlMove.fPitch, joyCtrlRef_.ctrlMove.fYaw);
  // RCLCPP_INFO(parentNode_->get_logger(),"thr:(%.4lf)", joyCtrlRef_.ctrlMove.fThr);
  // RCLCPP_INFO(parentNode_->get_logger(),"rp_accum:(%.4lf,%.4lf)", joyCtrlRef_.ctrlAuxMove.fRoll,
  // joyCtrlRef_.ctrlAuxMove.fPitch); RCLCPP_INFO(parentNode_->get_logger(),"autojoy:(%d,%d)",
  // (int)(joyCtrlRef_.bCtrlAutoMode), (int)(joyCtrlRef_.bCtrlJoyMode));
  // RCLCPP_INFO(parentNode_->get_logger(),"rd_gear:(%d,%d)", joyCtrlRef_.nRgearStatus, joyCtrlRef_.nDgearStatus);
  // RCLCPP_INFO(parentNode_->get_logger()," ");
}

// generating the joystick control reference information, buttons
JoyCtrlCmd JoyXBox360::GenJoyCtrlBtnsRefInfo(JoyRaw param, const sensor_msgs::msg::Joy::SharedPtr joyRaw,
                                             JoyCtrlCmd res)
{
  if ((joyRaw->buttons[param.ctrlButton.nTrigAutoCtrl] == 1) && (joyRaw->buttons[param.ctrlButton.nTrigJoyCtrl] == 0))
  {
    // using guidance control input
    res.bCtrlAutoMode = true;
    res.bCtrlJoyMode = false;
  }
  else if ((joyRaw->buttons[param.ctrlButton.nTrigAutoCtrl] == 0) &&
           (joyRaw->buttons[param.ctrlButton.nTrigJoyCtrl] == 1))
  {
    // using joystick control input
    res.bCtrlAutoMode = false;
    res.bCtrlJoyMode = true;
  }
  else if ((joyRaw->buttons[param.ctrlButton.nTrigAutoCtrl] == 1) &&
           (joyRaw->buttons[param.ctrlButton.nTrigJoyCtrl] == 1))
  {
    // default: using joystick control input
    res.bCtrlAutoMode = false;
    res.bCtrlJoyMode = true;
  }
  else
  {
    // staying the flag type
    res.bCtrlAutoMode = bPrevUseExtGuidLoop_;
    res.bCtrlJoyMode = bPrevUseJoyConLoop_;
  }

  // saving the previous data
  bPrevUseExtGuidLoop_ = res.bCtrlAutoMode;
  bPrevUseJoyConLoop_ = res.bCtrlJoyMode;

  // assigning gear status
  res.nRgearStatus = joyRaw->buttons[param.ctrlButton.nTrigRgear];
  res.nDgearStatus = joyRaw->buttons[param.ctrlButton.nTrigDgear];
  return res;
}

// generating the joystick control reference information, axes
JoyCtrlCmd JoyXBox360::GenJoyCtrlAxisRefInfo(JoyRaw param, const sensor_msgs::msg::Joy::SharedPtr joyRaw,
                                             JoyCtrlCmd res)
{
  res.ctrlMove.fRoll = (param.ctrlMove.fRollAxis) * (joyRaw->axes[param.ctrlMove.nRollChannel]);
  res.ctrlMove.fPitch = (param.ctrlMove.fPitchAxis) * (joyRaw->axes[param.ctrlMove.nPitchChannel]);
  res.ctrlMove.fYaw = (param.ctrlMove.fYawAxis) * (joyRaw->axes[param.ctrlMove.nYawChannel]);
  res.ctrlMove.fThr = (param.ctrlMove.fThrAxis) * (joyRaw->axes[param.ctrlMove.nThrChannel]);
  res.ctrlAuxMove.fRoll += (param.ctrlScale.fPhiAuxAccumScale) * (param.ctrlAuxMove.fRollAxis) *
                           (joyRaw->axes[param.ctrlAuxMove.nRollChannel]);
  res.ctrlAuxMove.fPitch += (param.ctrlScale.fThetaAuxAccumScale) * (param.ctrlAuxMove.fPitchAxis) *
                            (joyRaw->axes[param.ctrlAuxMove.nPitchChannel]);
  res.ctrlAuxMove.fYawLt = (param.ctrlAuxMove.fYawLtAxis) * (joyRaw->axes[param.ctrlAuxMove.nYawLtChannel]);
  res.ctrlAuxMove.fYawRt = (param.ctrlAuxMove.fYawRtAxis) * (joyRaw->axes[param.ctrlAuxMove.nYawRtChannel]);
  res.ctrlAuxMove.fRoll = misc_.Sat((res.ctrlAuxMove.fRoll), (-1.0f) * (param.ctrlScale.fPhiAuxAccumMax),
                                    (1.0f) * (param.ctrlScale.fPhiAuxAccumMax));
  res.ctrlAuxMove.fPitch = misc_.Sat((res.ctrlAuxMove.fPitch), (-1.0f) * (param.ctrlScale.fThetaAuxAccumMax),
                                     (1.0f) * (param.ctrlScale.fThetaAuxAccumMax));
  res.ctrlAuxMove.fYawLt = misc_.MapRange(res.ctrlAuxMove.fYawLt, -1.0, 1.0, param.ctrlScale.fPsiAuxMax,
                                          param.ctrlScale.fPsiAuxMin, param.ctrlScale.nPsiResol);
  res.ctrlAuxMove.fYawRt = misc_.MapRange(res.ctrlAuxMove.fYawRt, -1.0, 1.0, param.ctrlScale.fPsiAuxMax,
                                          param.ctrlScale.fPsiAuxMin, param.ctrlScale.nPsiResol);
  res.ctrlScale = param.ctrlScale;
  return res;
}