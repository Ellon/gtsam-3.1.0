/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ParallaxAnglePoint.cpp
 * @brief  Parallax Angle Point without depth
 * @author Ellon P. Mendes
 */

#include "ParallaxAnglePoint2.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void ParallaxAnglePoint2::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */
bool ParallaxAnglePoint2::equals(const ParallaxAnglePoint2 & q, double tol) const {
  return (fabs(yaw_ - q.yaw()) < tol &&
          fabs(pitch_ - q.pitch()) < tol);
}

/* ************************************************************************* */
ParallaxAnglePoint2 ParallaxAnglePoint2::operator+(const ParallaxAnglePoint2& q) const {
  return ParallaxAnglePoint2(yaw_ + q.yaw_, pitch_ + q.pitch_);
}

/* ************************************************************************* */
ParallaxAnglePoint2 ParallaxAnglePoint2::operator-(const ParallaxAnglePoint2& q) const {
  return ParallaxAnglePoint2(yaw_ - q.yaw_, pitch_ - q.pitch_);
}

/* ************************************************************************* */
Vector3 ParallaxAnglePoint2::directionVector(boost::optional<gtsam::Matrix&> H) const
{
  double sy = sin(yaw_), cy = cos(yaw_), sp = sin(pitch_), cp = cos(pitch_);
  if(H)
  {
    H->resize(3,2);
    *H << -sy*cp, -cy*sp,
           cy*cp, -sy*sp,
             0  ,   cp  ;
  }
  return (Vector(3) << cy*cp, sy*cp, sp);
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const ParallaxAnglePoint2& p) {
  os << '[' << p.yaw() << ", " << p.pitch() << "]\';";
  return os;
}

/* ************************************************************************* */
} // namespace gtsam
