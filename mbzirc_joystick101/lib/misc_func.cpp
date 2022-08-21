#include "misc_func.h"

using namespace std;
using namespace Eigen;

MiscFunc::MiscFunc(const ConfigParam& cfg) : cfgParam_(cfg)
{
}

MiscFunc::~MiscFunc()
{
}

// calculating the position error, NED frame, 2D space
Vector2d MiscFunc::Calc2DPosErrInfo(Vector2d pos2DNedRef, Vector2d pos2DNedCurr, double dTrjHeading)
{
  Vector2d res;
  res(0) = (-cos(WrapD(dTrjHeading)) * (pos2DNedCurr(0) - pos2DNedRef(0)) +
            sin(WrapD(dTrjHeading)) * (pos2DNedCurr(1) - pos2DNedRef(1)));
  res(1) = (-sin(WrapD(dTrjHeading)) * (pos2DNedCurr(0) - pos2DNedRef(0)) -
            cos(WrapD(dTrjHeading)) * (pos2DNedCurr(1) - pos2DNedRef(1)));
  return res;
}

// converting the Euler angle(3-2-1, ZYX, YPR) [rad] to the quaternion
Quaterniond MiscFunc::CalcQuaternionFromYPREulerAng(Vector3d euler)
{
  tf2::Matrix3x3 matQuat;
  tf2::Quaternion quat;
  Quaterniond result;
  matQuat.setEulerYPR(euler(2), euler(1), euler(0));
  matQuat.getRotation(quat);
  result.vec().x() = (double)(quat.x());
  result.vec().y() = (double)(quat.y());
  result.vec().z() = (double)(quat.z());
  result.w() = (double)(quat.w());
  return result;
}

// converting the quaternion to the Euler angle(3-2-1, ZYX, YPR) [rad]
Vector3d MiscFunc::CalcYPREulerAngFromQuaternion(Quaterniond q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 matQuat(quat);
  Vector3d result;
  double dYaw, dPitch, dRoll = 0.0;
  matQuat.getEulerYPR(dYaw, dPitch, dRoll);
  result(0) = WrapD(dRoll);
  result(1) = WrapD(dPitch);
  result(2) = WrapD(dYaw);
  return result;
}

// calculating DCM, from NED to Body, using Euler angle (3->2->1)
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d MiscFunc::CalcDcmNtoB(Vector3d eulerAtt)
{
  Matrix3d result;
  double cx = cos(eulerAtt(0));
  double cy = cos(eulerAtt(1));
  double cz = cos(eulerAtt(2));
  double sx = sin(eulerAtt(0));
  double sy = sin(eulerAtt(1));
  double sz = sin(eulerAtt(2));
  result(0, 0) = cy * cz;
  result(0, 1) = cy * sz;
  result(0, 2) = -sy;
  result(1, 0) = sy * sx * cz - sz * cx;
  result(1, 1) = sy * sx * sz + cz * cx;
  result(1, 2) = cy * sx;
  result(2, 0) = sy * cx * cz + sz * sx;
  result(2, 1) = sy * cx * sz - cz * sx;
  result(2, 2) = cy * cx;
  return result;
}

// calculating DCM, from Body to NED, using Euler angle (3->2->1)
Matrix3d MiscFunc::CalcDcmBtoN(Vector3d eulerAtt)
{
  Matrix3d result;
  Matrix3d DcmNtoB;
  DcmNtoB = CalcDcmNtoB(eulerAtt);
  result = DcmNtoB.inverse();
  return result;
}

// calculating DCM, Euler angle, 321 conversion (ref:from NED to Body, using Euler angle (3->2->1))
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d MiscFunc::CalcDcmEuler321(Vector3d eulerAtt)
{
  return CalcDcmNtoB(eulerAtt);
}

// converting from the position w.r.t ENU frame to the position w.r.t NED frame
Vector3d MiscFunc::ConvertPosFromEnuToNed(Vector3d posEnu)
{
  Vector3d result;
  result = (CalcDcmEnuToNed()) * (posEnu);
  return result;
}

// converting from the position w.r.t ENU frame to the position w.r.t NED frame
Vector3d MiscFunc::ConvertPosFromNedToEnu(Vector3d posNed)
{
  Vector3d result;
  result = ((CalcDcmEnuToNed()).inverse()) * (posNed);
  return result;
}

// calculating DCM, from ENU to NED
Matrix3d MiscFunc::CalcDcmEnuToNed()
{
  Vector3d attForEnuToNed;
  Matrix3d result;
  attForEnuToNed(0) = (0.0) * (D2R);
  attForEnuToNed(1) = (-180.0) * (D2R);
  attForEnuToNed(2) = (180.0) * (D2R);
  result = CalcDcmEuler321(attForEnuToNed);
  return result;
}

// calculating DCM, from NED to ENU
Matrix3d MiscFunc::CalcDcmNedToEnu()
{
  return CalcDcmEnuToNed().inverse();
}

// calculating DCM, for heading control using Euler angle (3->2->1)
Matrix3d MiscFunc::CalcDcmHeadingCtrl(float fYawRef)
{
  Matrix3d result;
  Matrix3d DcmNtoB;
  Vector3d eulerRef;
  eulerRef(0) = 0.0;
  eulerRef(1) = 0.0;
  eulerRef(2) = fYawRef;
  DcmNtoB = CalcDcmNtoB(eulerRef);
  result = DcmNtoB.inverse();
  return result;
}

// wrap-up function, angle between -PI and PI
double MiscFunc::WrapD(double _angle)
{
  _angle = fmod(_angle, 2.0 * PI);

  if (_angle < -PI)
  {
    _angle += 2.0 * PI;
  }
  else if (_angle > PI)
  {
    _angle -= 2.0 * PI;
  }
  else
  {
    _angle = _angle;
  }

  return _angle;
}

// generating local time to string with facet
string MiscFunc::GenLocalTimeStringFacet()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::microsec_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  auto facet = new boost::posix_time::time_facet("%Y %m %d %H %M %s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << currentTime.local_time();

  return ss.str();
}

// generating local time to string without facet
string MiscFunc::GenLocalTimeStringNormal()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::second_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  ss << currentTime.local_time();

  return ss.str();
}

double MiscFunc::MapRange(double sourceNumber, double fromA, double fromB, double toA, double toB, int decimalPrecision)
{
  double deltaA = fromB - fromA;
  double deltaB = toB - toA;
  double scale = deltaB / deltaA;
  double negA = -1 * fromA;
  double offset = (negA * scale) + toA;
  double finalNumber = (sourceNumber * scale) + offset;
  int calcScale = (int)(pow(10, decimalPrecision));
  return (double)(round(finalNumber * calcScale) / calcScale);
}

int MiscFunc::Sat(int nVal, int nMin, int nMax)
{
  int nRes = 0;
  if (nVal > nMax)
    nRes = nMax;
  else if (nVal < nMin)
    nRes = nMin;
  else
    nRes = nVal;
  return nRes;
}

float MiscFunc::Sat(float fVal, float fMin, float fMax)
{
  float fRes = 0.0f;
  if (fVal > fMax)
    fRes = fMax;
  else if (fVal < fMin)
    fRes = fMin;
  else
    fRes = fVal;
  return fRes;
}

double MiscFunc::Sat(double dVal, double dMin, double dMax)
{
  double dRes = 0.0;
  if (dVal > dMax)
    dRes = dMax;
  else if (dVal < dMin)
    dRes = dMin;
  else
    dRes = dVal;
  return dRes;
}
