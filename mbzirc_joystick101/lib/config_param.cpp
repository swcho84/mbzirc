#include "config_param.h"

using namespace std;
using namespace rclcpp;

ConfigParam::ConfigParam(const std::shared_ptr<rclcpp::Node>& parentNode) : parentNode_(parentNode)
{
  int nXbox360AuxilChannelPitch = 0;
  parentNode_->get_parameter("Joystick.XBox360.auxil.channel.pitch", nXbox360AuxilChannelPitch);
  printf("nXbox360AuxilChannelPitch:%d\n", nXbox360AuxilChannelPitch);
}

ConfigParam::~ConfigParam()
{
}

// reading rosparams (public function)
bool ConfigParam::GetRosParams()
{
  return ReadRosParams();
}

// reading rosparams (private function)
bool ConfigParam::ReadRosParams()
{
  try
  {
    // general information
    strHomeName_ = getenv("HOME");

    // reading parameters
    ReadRosParam("Joystick.joyTopicName", strJoyTpNm);

    ReadRosParam("Joystick.commandParam.scaleRPAtt.roll", joyMoveScale_.fAttPhi);
    ReadRosParam("Joystick.commandParam.scaleRPAtt.pitch", joyMoveScale_.fAttTheta);
    ReadRosParam("Joystick.commandParam.scaleYawRate", joyMoveScale_.fAttRatePsi);
    ReadRosParam("Joystick.commandParam.scaleHeaveVel", joyMoveScale_.fVelHeave);
    ReadRosParam("Joystick.commandParam.scaleRPAuxAttAccum.roll", joyMoveScale_.fPhiAuxAccumScale);
    ReadRosParam("Joystick.commandParam.scaleRPAuxAttAccum.pitch", joyMoveScale_.fThetaAuxAccumScale);
    ReadRosParam("Joystick.commandParam.scaleRPAuxAttMax.roll", joyMoveScale_.fPhiAuxAccumMax);
    ReadRosParam("Joystick.commandParam.scaleRPAuxAttMax.pitch", joyMoveScale_.fThetaAuxAccumMax);
    ReadRosParam("Joystick.commandParam.scaleYawAuxMinMaxResol.min", joyMoveScale_.fPsiAuxMin);
    ReadRosParam("Joystick.commandParam.scaleYawAuxMinMaxResol.max", joyMoveScale_.fPsiAuxMax);
    ReadRosParam("Joystick.commandParam.scaleYawAuxMinMaxResol.resol", joyMoveScale_.nPsiResol);
    ReadRosParam("Joystick.commandParam.scaleThrottleMinMaxResol.min", joyMoveScale_.fThrAuxMin);
    ReadRosParam("Joystick.commandParam.scaleThrottleMinMaxResol.max", joyMoveScale_.fThrAuxMax);
    ReadRosParam("Joystick.commandParam.scaleThrottleMinMaxResol.resol", joyMoveScale_.nThrResol);
    ReadRosParam("Joystick.commandParam.scaleYawAtt.yaw", fAttPsi);
    fAttPsi = (fAttPsi) * (D2R);
    joyMoveScale_.fAttPhi = (joyMoveScale_.fAttPhi) * (D2R);
    joyMoveScale_.fAttTheta = (joyMoveScale_.fAttTheta) * (D2R);
    joyMoveScale_.fAttRatePsi = (joyMoveScale_.fAttRatePsi) * (D2R);

    // reading parameters, w.r.t the xbox360 wired controller
    ReadRosParam("Joystick.XBox360.channel.roll", joyCfgXbox.ctrlMove.nRollChannel);
    ReadRosParam("Joystick.XBox360.channel.pitch", joyCfgXbox.ctrlMove.nPitchChannel);
    ReadRosParam("Joystick.XBox360.channel.yaw", joyCfgXbox.ctrlMove.nYawChannel);
    ReadRosParam("Joystick.XBox360.channel.throttle", joyCfgXbox.ctrlMove.nThrChannel);
    ReadRosParam("Joystick.XBox360.auxil.channel.roll", joyCfgXbox.ctrlAuxMove.nRollChannel);
    ReadRosParam("Joystick.XBox360.auxil.channel.pitch", joyCfgXbox.ctrlAuxMove.nPitchChannel);
    ReadRosParam("Joystick.XBox360.auxil.channel.yawleft", joyCfgXbox.ctrlAuxMove.nYawLtChannel);
    ReadRosParam("Joystick.XBox360.auxil.channel.yawright", joyCfgXbox.ctrlAuxMove.nYawRtChannel);
    ReadRosParam("Joystick.XBox360.auxil.direction.yawleft", joyCfgXbox.ctrlAuxMove.fYawLtAxis);
    ReadRosParam("Joystick.XBox360.auxil.direction.yawright", joyCfgXbox.ctrlAuxMove.fYawRtAxis);
    ReadRosParam("Joystick.XBox360.direction.roll", joyCfgXbox.ctrlMove.fRollAxis);
    ReadRosParam("Joystick.XBox360.direction.pitch", joyCfgXbox.ctrlMove.fPitchAxis);
    ReadRosParam("Joystick.XBox360.direction.yaw", joyCfgXbox.ctrlMove.fYawAxis);
    ReadRosParam("Joystick.XBox360.direction.throttle", joyCfgXbox.ctrlMove.fThrAxis);
    ReadRosParam("Joystick.XBox360.button.triggerAuto", joyCfgXbox.ctrlButton.nTrigAutoCtrl);
    ReadRosParam("Joystick.XBox360.button.triggerJoy", joyCfgXbox.ctrlButton.nTrigJoyCtrl);
    ReadRosParam("Joystick.XBox360.button.triggerRgear", joyCfgXbox.ctrlButton.nTrigRgear);
    ReadRosParam("Joystick.XBox360.button.triggerDgear", joyCfgXbox.ctrlButton.nTrigDgear);
    joyCfgXbox.ctrlAuxMove.fRollAxis = joyCfgXbox.ctrlMove.fRollAxis;
    joyCfgXbox.ctrlAuxMove.fPitchAxis = joyCfgXbox.ctrlMove.fPitchAxis;
    joyCfgXbox.ctrlScale = joyMoveScale_;
  }
  catch (RosParamNotFoundException& ex)
  {
    RCLCPP_ERROR(parentNode_->get_logger(), "Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }
  return true;
}

void ConfigParam::ReadRosParam(const string& key, float& val)
{
  if (!parentNode_->has_parameter(key))
    throw RosParamNotFoundException(key);
  parentNode_->get_parameter(key, val);
}

void ConfigParam::ReadRosParam(const string& key, double& val)
{
  if (!parentNode_->has_parameter(key))
    throw RosParamNotFoundException(key);
  parentNode_->get_parameter(key, val);
}

void ConfigParam::ReadRosParam(const string& key, bool& val)
{
  if (!parentNode_->has_parameter(key))
    throw RosParamNotFoundException(key);
  parentNode_->get_parameter(key, val);
}

void ConfigParam::ReadRosParam(const string& key, int32_t& val)
{
  if (!parentNode_->has_parameter(key))
    throw RosParamNotFoundException(key);
  parentNode_->get_parameter(key, val);
}

void ConfigParam::ReadRosParam(const string& key, string& val)
{
  if (!parentNode_->has_parameter(key))
    throw RosParamNotFoundException(key);
  parentNode_->get_parameter(key, val);
  if (val.empty())
    throw RosParamNotFoundException(key);
}