#ifndef MBZIRC_JOYSTICK101_MISC_FUNC_H
#define MBZIRC_JOYSTICK101_MISC_FUNC_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace Eigen;

class MiscFunc
{
public:
  MiscFunc(const ConfigParam& cfg);
  ~MiscFunc();

  Vector2d Calc2DPosErrInfo(Vector2d pos2DNedRef, Vector2d pos2DNedCurr, double dTrjHeading);
  Quaterniond CalcQuaternionFromYPREulerAng(Vector3d euler);
  Vector3d CalcYPREulerAngFromQuaternion(Quaterniond q);
  Matrix3d CalcDcmNtoB(Vector3d eulerAtt);
  Matrix3d CalcDcmBtoN(Vector3d eulerAtt);
  Matrix3d CalcDcmHeadingCtrl(float fYawRef);
  Matrix3d CalcDcmEuler321(Vector3d eulerAtt);
  Vector3d ConvertPosFromEnuToNed(Vector3d posEnu);
  Vector3d ConvertPosFromNedToEnu(Vector3d posNed);
  Matrix3d CalcDcmEnuToNed();
  Matrix3d CalcDcmNedToEnu();
  double WrapD(double _angle);

  string GenLocalTimeStringNormal();
  string GenLocalTimeStringFacet();
  double MapRange(double sourceNumber, double fromA, double fromB, double toA, double toB, int decimalPrecision);
  int Sat(int nVal, int nMin, int nMax);
  float Sat(float fVal, float fMin, float fMax);
  double Sat(double dVal, double dMin, double dMax);

private:
  ConfigParam cfgParam_;
};

#endif  // MBZIRC_JOYSTICK101_MISC_FUNC_H