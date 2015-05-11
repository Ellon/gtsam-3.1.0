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

// Prototypes of local functions
double angleBetweenUnitVectors(
  const gtsam::Vector3 & v1, const gtsam::Vector3 & v2,
  boost::optional<gtsam::Matrix&> Dv1 = boost::none,
  boost::optional<gtsam::Matrix&> Dv2 = boost::none);

double norm(gtsam::Vector3 v, boost::optional<gtsam::Matrix&> Dv = boost::none);

namespace gtsam {
/* ************************************************************************* */
ParallaxAnglePoint3 ParallaxAnglePoint3::FromParallaxAnglePointAndAnchors(
  const ParallaxAnglePoint3 &oldPoint,
  const Point3 &oldMainAnchor, const Point3 &oldAssoAnchor,
  const Point3 &newMainAnchor, const Point3 &newAssoAnchor)
{
  Vector3 vecFromNewMain(oldPoint.directionVectorFromOtheAnchor(oldMainAnchor, oldAssoAnchor, newMainAnchor));
  Vector3 vecFromNewAsso(oldPoint.directionVectorFromOtheAnchor(oldMainAnchor, oldAssoAnchor, newAssoAnchor));

  double yaw   = atan2(vecFromNewMain.y(),vecFromNewMain.x());
  double pitch = atan2(vecFromNewMain.z(),Point2(vecFromNewMain.x(),vecFromNewMain.y()).norm());
  double parallax = acos(vecFromNewMain.dot(vecFromNewAsso)/(vecFromNewMain.norm()*vecFromNewAsso.norm()));

  return ParallaxAnglePoint3(yaw,pitch,parallax);
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
ostream &operator<<(ostream &os, const ParallaxAnglePoint3& p) {
  os << '(' << p.yaw() << ", " << p.pitch() << ", " << p.parallax() << ')';
  return os;
}

/* ************************************************************************* */
Vector3 ParallaxAnglePoint3::directionVectorFromMainAnchor(
  boost::optional<gtsam::Matrix&> H) const
{
  double sy = sin(yaw_), cy = cos(yaw_), sp = sin(pitch_), cp = cos(pitch_);
  if(H)
  {
    H->resize(3,3);
    *H << -sy*cp, -cy*sp, 0,
           cy*cp, -sy*sp, 0,
             0  ,   cp  , 0;
  }
  return (Vector(3) << cy*cp, sy*cp, sp);
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
    Point3 main_T_asso(assoAnchor - mainAnchor);

    // Get a normalized version of it
    Vector3 main_T_asso_normd(main_T_asso.normalize().vector());

    // Get the lenght of the vector
    double norm_mTa = main_T_asso.norm();

    // Get direction vector from main anchor to point
    // NOTE: It's already normalized
    Vector3 dirvec_mTp(directionVectorFromMainAnchor());

    // Get the angle between these vectors
    double phi = angleBetweenUnitVectors(dirvec_mTp, main_T_asso_normd);

    // Get vector from main anchor to other anchor
    Vector3 main_T_other((otheAnchor - mainAnchor).vector());

    return sin(parallax_ + phi) * norm_mTa * dirvec_mTp - sin(parallax_) * main_T_other;
  }

  // Same computation with jacobians

  // Get vector from main anchor to associated anchor
  Matrix MTA_assoAnchor, MTA_mainAnchor;
  Point3 main_T_asso(assoAnchor.sub(mainAnchor, MTA_assoAnchor, MTA_mainAnchor));

  // Get a normalized version of it
  Matrix MTANORMD_mta;
  Vector3 main_T_asso_normd(main_T_asso.normalize(MTANORMD_mta).vector());

  // Get the lenght of the vector
  Matrix NORMMTA_mta;
  double norm_mTa = norm(main_T_asso.vector(), NORMMTA_mta);

  // Get direction vector from main anchor to point
  // NOTE: It's already normalized
  Matrix DIRVECMTP_point;
  Vector3 dirvec_mTp(directionVectorFromMainAnchor(DIRVECMTP_point));

  // Get the angle between these vectors
  Matrix PHI_dirvecmtp, PHI_mtanormd;
  double phi = angleBetweenUnitVectors(dirvec_mTp, main_T_asso_normd, PHI_dirvecmtp, PHI_mtanormd);

  // Get vector from main anchor to other anchor
  Matrix MTO_otheanchor, MTO_mainanchor;
  Vector3 main_T_othe(otheAnchor.sub(mainAnchor, MTO_otheanchor, MTO_mainanchor).vector());

  // Chain of partial derivates
  // main anchor
  if(Dmain)
  {
    *Dmain = dirvec_mTp * (  cos(parallax_ + phi) * norm_mTa * PHI_mtanormd * MTANORMD_mta * MTA_mainAnchor
                           + sin(parallax_ + phi) * NORMMTA_mta * MTA_mainAnchor                           )
             - sin(parallax_) * MTO_mainanchor;
  }
  // associated anchor
  if(Dasso)
  {
    *Dasso = dirvec_mTp * (  cos(parallax_ + phi) * norm_mTa * PHI_mtanormd * MTANORMD_mta * MTA_assoAnchor
                           + sin(parallax_ + phi) * NORMMTA_mta * MTA_assoAnchor                           );
  }
  // other anchor
  if(Dothe)
  {
    *Dothe = - sin(parallax_) * MTO_otheanchor;
  }
  // parallax point
  if(Dpoint)
  {
    Dpoint->resize(3,3);
    Dpoint->block(0,0,3,2) = (dirvec_mTp * (cos(parallax_ + phi)*PHI_dirvecmtp*DIRVECMTP_point.block(0,0,3,2)) + sin(parallax_ + phi)*DIRVECMTP_point.block(0,0,3,2)) * norm_mTa;
    Dpoint->block(0,2,3,1) = norm_mTa * dirvec_mTp * cos(parallax_ + phi) - main_T_othe * cos(parallax_);
  }

  return sin(parallax_ + phi) * norm_mTa * dirvec_mTp - sin(parallax_) * main_T_othe;

}

}

// Local Functions
double angleBetweenUnitVectors(
  const gtsam::Vector3 & v1, const gtsam::Vector3 & v2,
  boost::optional<gtsam::Matrix&> Dv1,
  boost::optional<gtsam::Matrix&> Dv2)
{
  double dot_prod = v1.dot(v2);
  if(Dv1)
  {
    Dv1->resize(1,3);
    *Dv1 = -(1/sqrt(1-(dot_prod*dot_prod))) * v2.transpose();
  }
 if(Dv2)
  {
    Dv2->resize(1,3);
    *Dv2 = -(1/sqrt(1-(dot_prod*dot_prod))) * v1.transpose();
  }
  return acos(dot_prod);
}

double norm(gtsam::Vector3 v, boost::optional<gtsam::Matrix&> Dv)
{
  double n = v.norm();
  if(Dv)
  {
    Dv->resize(1,3);
    *Dv << v(0)/n, v(1)/n, v(2)/n;
  }
  return n;
}
