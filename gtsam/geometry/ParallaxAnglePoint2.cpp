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
  if(H)
  {
    Matrix VEC_p(3,1), VEC_y(3,1);
    Vector3 vec = py2vec(pitch(),yaw(),VEC_p,VEC_y);

    H->resize(3,2);
    *H << VEC_p, VEC_y;
    return vec;
  }

  return py2vec(pitch(), yaw());
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const ParallaxAnglePoint2& p) {
  os << '(' << p.yaw() << ", " << p.pitch() << ')';
  return os;
}

/* ************************************************************************* */
} // namespace gtsam
