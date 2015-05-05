/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.cpp
 * @date Feb 28, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3DS2.h>

namespace gtsam {

/* ************************************************************************* */
Cal3DS2::Cal3DS2(const Vector &v):
    fx_(v[0]), fy_(v[1]), s_(v[2]), u0_(v[3]), v0_(v[4]), k1_(v[5]), k2_(v[6]), k3_(v[7]), k4_(v[8]){}

/* ************************************************************************* */
Matrix Cal3DS2::K() const {
  return (Matrix(3, 3) << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0);
}

/* ************************************************************************* */
Vector Cal3DS2::vector() const {
  return (Vector(9) << fx_, fy_, s_, u0_, v0_, k1_, k2_, k3_, k4_);
}

/* ************************************************************************* */
void Cal3DS2::print(const std::string& s) const {
  gtsam::print(K(), s + ".K");
  gtsam::print(Vector(k()), s + ".k");
}

/* ************************************************************************* */
bool Cal3DS2::equals(const Cal3DS2& K, double tol) const {
  if (fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol || fabs(s_ - K.s_) > tol ||
      fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol || fabs(k1_ - K.k1_) > tol ||
      fabs(k2_ - K.k2_) > tol || fabs(k3_ - K.k3_) > tol || fabs(k4_ - K.k4_) > tol)
    return false;
  return true;
}

/* ************************************************************************* */
Point2 Cal3DS2::uncalibrate(const Point2& p,
       boost::optional<Matrix&> H1,
       boost::optional<Matrix&> H2) const {

  // parameters
  const double fx = fx_, fy = fy_, s = s_;
  const double k1 = k1_, k2 = k2_, k3 = k3_, k4 = k4_;

  //  rr = x^2 + y^2;
  //  g = (1 + k(1)*rr + k(2)*rr^2);
  //  dp = [2*k(3)*x*y + k(4)*(rr + 2*x^2); 2*k(4)*x*y + k(3)*(rr + 2*y^2)];
  //  pi(:,i) = g * pn(:,i) + dp;
  const double x = p.x(), y = p.y(), xy = x * y, xx = x * x, yy = y * y;
  const double rr = xx + yy;
  const double r4 = rr * rr;
  const double g = 1. + k1 * rr + k2 * r4;
  const double dx = 2. * k3 * xy + k4 * (rr + 2. * xx);
  const double dy = 2. * k4 * xy + k3 * (rr + 2. * yy);

  const double pnx = g*x + dx;
  const double pny = g*y + dy;

  // Inlined derivative for calibration
  if (H1) {
    *H1 = (Matrix(2, 9) <<  pnx, 0.0, pny, 1.0, 0.0, fx * x * rr + s * y * rr,
        fx * x * r4 + s * y * r4, fx * 2. * xy + s * (rr + 2. * yy),
        fx * (rr + 2. * xx) + s * (2. * xy), 0.0, pny, 0.0, 0.0, 1.0,
        fy * y * rr, fy * y * r4, fy * (rr + 2. * yy), fy * (2. * xy));
  }
  // Inlined derivative for points
  if (H2) {
    const double dr_dx = 2. * x;
    const double dr_dy = 2. * y;
    const double dg_dx = k1 * dr_dx + k2 * 2. * rr * dr_dx;
    const double dg_dy = k1 * dr_dy + k2 * 2. * rr * dr_dy;

    const double dDx_dx = 2. * k3 * y + k4 * (dr_dx + 4. * x);
    const double dDx_dy = 2. * k3 * x + k4 * dr_dy;
    const double dDy_dx = 2. * k4 * y + k3 * dr_dx;
    const double dDy_dy = 2. * k4 * x + k3 * (dr_dy + 4. * y);

    Matrix DK = (Matrix(2, 2) << fx, s_, 0.0, fy);
    Matrix DR = (Matrix(2, 2) << g + x * dg_dx + dDx_dx, x * dg_dy + dDx_dy,
        y * dg_dx + dDy_dx, g + y * dg_dy + dDy_dy);

    *H2 = DK * DR;
  }

  return Point2(fx * pnx + s * pny + u0_, fy * pny + v0_);
}

/* ************************************************************************* */
Point2 Cal3DS2::calibrate(const Point2& pi, const double tol) const {
  // Use the following fixed point iteration to invert the radial distortion.
  // pn_{t+1} = (inv(K)*pi - dp(pn_{t})) / g(pn_{t})

  const Point2 invKPi ((1 / fx_) * (pi.x() - u0_ - (s_ / fy_) * (pi.y() - v0_)),
                       (1 / fy_) * (pi.y() - v0_));

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for ( iteration = 0; iteration < maxIterations; ++iteration ) {
    if ( uncalibrate(pn).distance(pi) <= tol ) break;
    const double x = pn.x(), y = pn.y(), xy = x*y, xx = x*x, yy = y*y;
    const double rr = xx + yy;
    const double g = (1+k1_*rr+k2_*rr*rr);
    const double dx = 2*k3_*xy + k4_*(rr+2*xx);
    const double dy = 2*k4_*xy + k3_*(rr+2*yy);
    pn = (invKPi - Point2(dx,dy))/g;
  }

  if ( iteration >= maxIterations )
    throw std::runtime_error("Cal3DS2::calibrate fails to converge. need a better initialization");

  return pn;
}

/* ************************************************************************* */
Matrix Cal3DS2::D2d_intrinsic(const Point2& p) const {
  //const double fx = fx_, fy = fy_, s = s_;
  const double k1 = k1_, k2 = k2_, k3 = k3_, k4 = k4_;
  //const double x = p.x(), y = p.y(), xx = x*x, yy = y*y, xy = x*y;
  const double x = p.x(), y = p.y(), xx = x*x, yy = y*y;
  const double rr = xx + yy;
  const double dr_dx = 2*x;
  const double dr_dy = 2*y;
  const double r4 = rr*rr;
  const double g = 1 + k1*rr + k2*r4;
  const double dg_dx = k1*dr_dx + k2*2*rr*dr_dx;
  const double dg_dy = k1*dr_dy + k2*2*rr*dr_dy;

  // Dx = 2*k3*xy + k4*(rr+2*xx);
  // Dy = 2*k4*xy + k3*(rr+2*yy);
  const double dDx_dx = 2*k3*y + k4*(dr_dx + 4*x);
  const double dDx_dy = 2*k3*x + k4*dr_dy;
  const double dDy_dx = 2*k4*y + k3*dr_dx;
  const double dDy_dy = 2*k4*x + k3*(dr_dy + 4*y);

  Matrix DK = (Matrix(2, 2) << fx_, s_, 0.0, fy_);
  Matrix DR = (Matrix(2, 2) << g + x*dg_dx + dDx_dx,     x*dg_dy + dDx_dy,
             y*dg_dx + dDy_dx, g + y*dg_dy + dDy_dy);

  return DK * DR;
}

/* ************************************************************************* */
Matrix Cal3DS2::D2d_calibration(const Point2& p) const {
  const double x = p.x(), y = p.y(), xx = x*x, yy = y*y, xy = x*y;
  const double rr = xx + yy;
  const double r4 = rr*rr;
  const double fx = fx_, fy = fy_, s = s_;
  const double g = (1+k1_*rr+k2_*r4);
  const double dx = 2*k3_*xy + k4_*(rr+2*xx);
  const double dy = 2*k4_*xy + k3_*(rr+2*yy);
  const double pnx = g*x + dx;
  const double pny = g*y + dy;

  return (Matrix(2, 9) <<
  pnx, 0.0, pny, 1.0, 0.0, fx*x*rr + s*y*rr, fx*x*r4 + s*y*r4, fx*2*xy + s*(rr+2*yy), fx*(rr+2*xx) + s*(2*xy),
  0.0, pny, 0.0, 0.0, 1.0, fy*y*rr      , fy*y*r4         , fy*(rr+2*yy)         , fy*(2*xy)  );
}

/* ************************************************************************* */
Cal3DS2 Cal3DS2::retract(const Vector& d) const {
  return Cal3DS2(vector() + d);
}

/* ************************************************************************* */
Vector Cal3DS2::localCoordinates(const Cal3DS2& T2) const {
  return T2.vector() - vector();
}

}
/* ************************************************************************* */


