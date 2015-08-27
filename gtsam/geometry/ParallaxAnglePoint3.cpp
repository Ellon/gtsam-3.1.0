/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ParallaxAnglePoint3.cpp
 * @brief  3D Parallax Angle Point
 * @author Ellon P. Mendes
 */

#include "ParallaxAnglePoint3.h"

using namespace std;

namespace gtsam {
/* ************************************************************************* */
ParallaxAnglePoint3 ParallaxAnglePoint3::FromParallaxAnglePointAndAnchors(
  const ParallaxAnglePoint3 &oldPoint,
  const Point3 &oldMainAnchor, const Point3 &oldAssoAnchor,
  const Point3 &newMainAnchor, const Point3 &newAssoAnchor)
{
  Vector3 vecFromNewMain(oldPoint.directionVectorFromOtheAnchor(oldMainAnchor, oldAssoAnchor, newMainAnchor));
  Vector3 vecFromNewAsso(oldPoint.directionVectorFromOtheAnchor(oldMainAnchor, oldAssoAnchor, newAssoAnchor));

  Vector2 py = vec2py(vecFromNewMain);
  double pitch = py(0);
  double yaw   = py(1);

  double parallax = vectors2angle(vecFromNewMain,vecFromNewAsso);

  return ParallaxAnglePoint3(pitch,yaw,parallax);
}

/* ************************************************************************* */
void ParallaxAnglePoint3::print(const string& s) const {
  cout << s << *this << endl;
}

/* ************************************************************************* */
bool ParallaxAnglePoint3::equals(const ParallaxAnglePoint3 & q, double tol) const {
  return (fabs(yaw_      - q.yaw()     ) < tol &&
          fabs(pitch_    - q.pitch()   ) < tol &&
          fabs(parallax_ - q.parallax()) < tol);
}

/* ************************************************************************* */
ParallaxAnglePoint3 ParallaxAnglePoint3::operator+(const ParallaxAnglePoint3& q) const {
  return ParallaxAnglePoint3(yaw_      + q.yaw_,
                             pitch_    + q.pitch_,
                             parallax_ + q.parallax_);
}

/* ************************************************************************* */
ParallaxAnglePoint3 ParallaxAnglePoint3::operator-(const ParallaxAnglePoint3& q) const {
  return ParallaxAnglePoint3(yaw_      - q.yaw_,
                             pitch_    - q.pitch_,
                             parallax_ - q.parallax_);
}

/* ************************************************************************* */
ParallaxAnglePoint3 ParallaxAnglePoint3::retract(const Vector& v) const
{
  ParallaxAnglePoint3 point = (*this + v);

  // Force parallax to be positive
  if(point.parallax_ < 1e-6)
    point.parallax_ = 1e-6;

  return point;
}

/* ************************************************************************* */
ostream &operator<<(ostream &os, const ParallaxAnglePoint3& p) {
  os << '(' << p.yaw() << ", " << p.pitch() << ", " << p.parallax() << ')';
  return os;
}

/* ************************************************************************* */
Vector3 ParallaxAnglePoint3::directionVectorFromMainAnchor(
  boost::optional<gtsam::Matrix&> H) const
{
  if(H)
  {
    Matrix VEC_p(3,1), VEC_y(3,1);
    Vector3 vec = py2vec(pitch(),yaw(),VEC_p,VEC_y);

    H->resize(3,3);
    *H << VEC_p, VEC_y, zeros(3,1);
    return vec;
  }

  return py2vec(pitch(), yaw());
}

/* ************************************************************************* */
Vector3 ParallaxAnglePoint3::directionVectorFromAssoAnchor(
  const Point3 & mainAnchor, const Point3 & assoAnchor,
  boost::optional<gtsam::Matrix&> Dpoint,
  boost::optional<gtsam::Matrix&> Dmain ,
  boost::optional<gtsam::Matrix&> Dasso ) const
{
  if (!Dasso)
  {
      return directionVectorFromOtheAnchor( mainAnchor, assoAnchor, assoAnchor, Dpoint, Dmain );
  }

  Matrix Dothe;
  Vector3 directVec = directionVectorFromOtheAnchor(
    mainAnchor, assoAnchor, assoAnchor,
    Dpoint, Dmain, Dasso, Dothe);

  *Dasso += Dothe;

  return directVec;

}

/* ************************************************************************* */
Vector3 ParallaxAnglePoint3::directionVectorFromOtheAnchor(
  const Point3 & mainAnchor, const Point3 & assoAnchor, const Point3 & otheAnchor,
  boost::optional<gtsam::Matrix&> Dpoint,
  boost::optional<gtsam::Matrix&> Dmain ,
  boost::optional<gtsam::Matrix&> Dasso ,
  boost::optional<gtsam::Matrix&> Dothe ) const
{

  if (!Dpoint && !Dmain && !Dasso && !Dothe)
  {
    // Get vector from main anchor to associated anchor
    Vector3 mainToAsso( assoAnchor.vector() - mainAnchor.vector() );

    // Get direction vector from main anchor to point
    Vector3 dirMainToPoint(directionVectorFromMainAnchor());

    // Get the angle between these vectors
    double phi = vectors2angle(dirMainToPoint, mainToAsso);

    // Get vector from main anchor to other anchor
    Vector3 mainToOther(otheAnchor.vector() - mainAnchor.vector());

    return sin(parallax_ + phi) * mainToAsso.norm() * dirMainToPoint - sin(parallax_) * mainToOther;
  }

  // Same computation with jacobians

  // Get vector from main anchor to associated anchor
  Matrix MAINTOASSO_assoanchor, MAINTOASSO_mainanchor;
  Vector3 mainToAsso( assoAnchor.sub(mainAnchor, MAINTOASSO_assoanchor, MAINTOASSO_mainanchor).vector() );

  // Get the lenght of the vector
  Matrix NORMMAINTOASSO_maintoasso;
  double normMainToAsso = norm(mainToAsso, NORMMAINTOASSO_maintoasso);

  // Get direction vector from main anchor to point
  // NOTE: It's already normalized
  Matrix DIRMAINTOPOINT_point;
  Vector3 dirMainToPoint(directionVectorFromMainAnchor(DIRMAINTOPOINT_point));

  // Get the angle between these vectors
  Matrix PHI_dirmaintopoint, PHI_maintoasso;
  double phi = vectors2angle(dirMainToPoint, mainToAsso, PHI_dirmaintopoint, PHI_maintoasso);

  // Get vector from main anchor to other anchor
  Matrix MAINTOOTHER_otheanchor, MAINTOOTHER_mainanchor;
  Vector3 mainToOther(otheAnchor.sub(mainAnchor, MAINTOOTHER_otheanchor, MAINTOOTHER_mainanchor).vector());

  double sinParallaxPhi = sin(parallax_ + phi);
  double SINPARALLAXPHI_parallax = cos(parallax_ + phi);
  double SINPARALLAXPHI_phi      = cos(parallax_ + phi);

  Vector3 dirOtherToPoint = sinParallaxPhi * normMainToAsso * dirMainToPoint - sin(parallax_) * mainToOther;

  // Chain of partial derivates
  // main anchor

  if(Dmain)
  {
    Dmain->resize(3,3);
    *Dmain << dirMainToPoint*( normMainToAsso*(SINPARALLAXPHI_phi * PHI_maintoasso * MAINTOASSO_mainanchor) + sinParallaxPhi*(NORMMAINTOASSO_maintoasso * MAINTOASSO_mainanchor) ) - sin(parallax_)*MAINTOOTHER_mainanchor;
  }
  // associated anchor
  if(Dasso)
  {
    Dasso->resize(3,3);
    *Dasso << dirMainToPoint*( normMainToAsso*(SINPARALLAXPHI_phi * PHI_maintoasso * MAINTOASSO_assoanchor) + sinParallaxPhi*(NORMMAINTOASSO_maintoasso * MAINTOASSO_assoanchor) );
  }
  // other anchor
  if(Dothe)
  {
    Dothe->resize(3,3);
    *Dothe = -sin(parallax_) * MAINTOOTHER_otheanchor;
  }
  // parallax point
  if(Dpoint)
  {
    Matrix DIROTHERTOPOINT_py(3,2);
    DIROTHERTOPOINT_py << normMainToAsso*( dirMainToPoint*(SINPARALLAXPHI_phi * PHI_dirmaintopoint * DIRMAINTOPOINT_point.block(0,0,3,2)) + sinParallaxPhi*DIRMAINTOPOINT_point.block(0,0,3,2) );

    Matrix DIROTHERTOPOINT_parallax(3,1);
    DIROTHERTOPOINT_parallax << (normMainToAsso * dirMainToPoint * SINPARALLAXPHI_parallax) - (cos(parallax_) * mainToOther);

    Dpoint->resize(3,3);
    *Dpoint << DIROTHERTOPOINT_py, DIROTHERTOPOINT_parallax;
  }

  return dirOtherToPoint;

}

Point3 ParallaxAnglePoint3::toPoint3(const Point3 & mainAnchor, const Point3 &  assoAnchor) const
{
  Vector3 vecFromMain = directionVectorFromMainAnchor();

  Point3 mainToAsso = (assoAnchor - mainAnchor);

  double mainAngle = vectors2angle(vecFromMain, mainToAsso.vector());

  double sinParalax = sin(parallax_);
  // Guard to avoid division by zero
  if(fabs(sinParalax) <= 1e-6)
  {
    if (sinParalax < 0) sinParalax = -1e-6;
    else sinParalax = 1e-6;
  }

  double depthFromMain = (sin(mainAngle + parallax_)/sinParalax)*mainToAsso.norm();

  return mainAnchor + Point3(vecFromMain*depthFromMain);

}

Point3 ParallaxAnglePoint3::toPoint3(const Pose3 & mainPose, const Pose3 & assoPose, boost::optional<const Pose3 &> body_P_sensor) const
{
  if(body_P_sensor)
  {
    return toPoint3(mainPose.compose(*body_P_sensor).translation(), assoPose.compose(*body_P_sensor).translation());
  }
  //else

  return toPoint3(mainPose.translation(), assoPose.translation());
}

}
