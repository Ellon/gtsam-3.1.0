/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3 class
 * @author  Alireza Fathi
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <boost/math/constants/constants.hpp>

#include <CppUnitLite/TestHarness.h>

#ifndef GTSAM_USE_QUATERNIONS

using namespace std;
using namespace gtsam;

static Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);
static Point3 P(0.2, 0.7, -2.0);
static double error = 1e-9, epsilon = 0.001;
static const Matrix I3 = eye(3);

/* ************************************************************************* */
TEST( Rot3, constructor)
{
  Rot3 expected(I3);
  Vector r1(3), r2(3), r3(3);
  r1(0) = 1;
  r1(1) = 0;
  r1(2) = 0;
  r2(0) = 0;
  r2(1) = 1;
  r2(2) = 0;
  r3(0) = 0;
  r3(1) = 0;
  r3(2) = 1;
  Rot3 actual(r1, r2, r3);
  CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor2)
{
  Matrix R = (Matrix(3, 3) << 11., 12., 13., 21., 22., 23., 31., 32., 33.);
  Rot3 actual(R);
  Rot3 expected(11, 12, 13, 21, 22, 23, 31, 32, 33);
  CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor3)
{
  Rot3 expected(1, 2, 3, 4, 5, 6, 7, 8, 9);
  Point3 r1(1, 4, 7), r2(2, 5, 8), r3(3, 6, 9);
  CHECK(assert_equal(Rot3(r1,r2,r3),expected));
}

/* ************************************************************************* */
TEST( Rot3, transpose)
{
  Rot3 R(1, 2, 3, 4, 5, 6, 7, 8, 9);
  Point3 r1(1, 2, 3), r2(4, 5, 6), r3(7, 8, 9);
  CHECK(assert_equal(R.inverse(),Rot3(r1,r2,r3)));
}

/* ************************************************************************* */
TEST( Rot3, equals)
{
  CHECK(R.equals(R));
  Rot3 zero;
  CHECK(!R.equals(zero));
}

/* ************************************************************************* */
// Notice this uses J^2 whereas fast uses w*w', and has cos(t)*I + ....
Rot3 slow_but_correct_rodriguez(const Vector& w) {
  double t = norm_2(w);
  Matrix J = skewSymmetric(w / t);
  if (t < 1e-5) return Rot3();
  Matrix R = I3 + sin(t) * J + (1.0 - cos(t)) * (J * J);
  return R;
}

/* ************************************************************************* */
TEST( Rot3, rodriguez)
{
  Rot3 R1 = Rot3::rodriguez(epsilon, 0, 0);
  Vector w = (Vector(3) << epsilon, 0., 0.);
  Rot3 R2 = slow_but_correct_rodriguez(w);
  CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez2)
{
  Vector axis = (Vector(3) << 0., 1., 0.); // rotation around Y
  double angle = 3.14 / 4.0;
  Rot3 actual = Rot3::rodriguez(axis, angle);
  Rot3 expected(0.707388, 0, 0.706825,
                       0, 1,        0,
               -0.706825, 0, 0.707388);
  CHECK(assert_equal(expected,actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez3)
{
  Vector w = (Vector(3) << 0.1, 0.2, 0.3);
  Rot3 R1 = Rot3::rodriguez(w / norm_2(w), norm_2(w));
  Rot3 R2 = slow_but_correct_rodriguez(w);
  CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez4)
{
  Vector axis = (Vector(3) << 0., 0., 1.); // rotation around Z
  double angle = M_PI/2.0;
  Rot3 actual = Rot3::rodriguez(axis, angle);
  double c=cos(angle),s=sin(angle);
  Rot3 expected(c,-s, 0,
                s, c, 0,
                0, 0, 1);
  CHECK(assert_equal(expected,actual,1e-5));
  CHECK(assert_equal(slow_but_correct_rodriguez(axis*angle),actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, expmap)
{
  Vector v = zero(3);
  CHECK(assert_equal(R.retract(v), R));
}

/* ************************************************************************* */
TEST(Rot3, log)
{
  static const double PI = boost::math::constants::pi<double>();
  Vector w;
  Rot3 R;

#define CHECK_OMEGA(X,Y,Z) \
  w = (Vector(3) << (double)X, (double)Y, double(Z)); \
  R = Rot3::rodriguez(w); \
  EXPECT(assert_equal(w, Rot3::Logmap(R),1e-12));

  // Check zero
  CHECK_OMEGA(  0,   0,   0)

  // create a random direction:
  double norm=sqrt(1.0+16.0+4.0);
  double x=1.0/norm, y=4.0/norm, z=2.0/norm;

  // Check very small rotation for Taylor expansion
  // Note that tolerance above is 1e-12, so Taylor is pretty good !
  double d = 0.0001;
  CHECK_OMEGA(  d,   0,   0)
  CHECK_OMEGA(  0,   d,   0)
  CHECK_OMEGA(  0,   0,   d)
  CHECK_OMEGA(x*d, y*d, z*d)

  // check normal rotation
  d = 0.1;
  CHECK_OMEGA(  d,   0,   0)
  CHECK_OMEGA(  0,   d,   0)
  CHECK_OMEGA(  0,   0,   d)
  CHECK_OMEGA(x*d, y*d, z*d)

  // Check 180 degree rotations
  CHECK_OMEGA(  PI,   0,   0)
  CHECK_OMEGA(   0,  PI,   0)
  CHECK_OMEGA(   0,   0,  PI)
  CHECK_OMEGA(x*PI,y*PI,z*PI)

  // Check 360 degree rotations
#define CHECK_OMEGA_ZERO(X,Y,Z) \
  w = (Vector(3) << (double)X, (double)Y, double(Z)); \
  R = Rot3::rodriguez(w); \
  EXPECT(assert_equal(zero(3), Rot3::Logmap(R)));

  CHECK_OMEGA_ZERO( 2.0*PI,      0,      0)
  CHECK_OMEGA_ZERO(      0, 2.0*PI,      0)
  CHECK_OMEGA_ZERO(      0,      0, 2.0*PI)
  CHECK_OMEGA_ZERO(x*2.*PI,y*2.*PI,z*2.*PI)
}

Rot3 evaluateRotation(const Vector3 thetahat){
  return Rot3::Expmap(thetahat);
}

Vector3 evaluateLogRotation(const Vector3 thetahat, const Vector3 deltatheta){
  return Rot3::Logmap( Rot3::Expmap(thetahat).compose( Rot3::Expmap(deltatheta) ) );
}

/* ************************************************************************* */
TEST( Rot3, rightJacobianExpMapSO3 )
{
  // Linearization point
  Vector3 thetahat; thetahat << 0.1, 0, 0;

  Matrix expectedJacobian = numericalDerivative11<Rot3, LieVector>(boost::bind(&evaluateRotation, _1), LieVector(thetahat));
  Matrix actualJacobian = Rot3::rightJacobianExpMapSO3(thetahat);
  EXPECT(assert_equal(expectedJacobian, actualJacobian));
}

/* ************************************************************************* */
TEST( Rot3, rightJacobianExpMapSO3inverse )
{
  // Linearization point
  Vector3 thetahat; thetahat << 0.1,0.1,0; ///< Current estimate of rotation rate bias
  Vector3 deltatheta; deltatheta << 0, 0, 0;

  Matrix expectedJacobian = numericalDerivative11<LieVector>(boost::bind(&evaluateLogRotation, thetahat, _1), LieVector(deltatheta));
  Matrix actualJacobian = Rot3::rightJacobianExpMapSO3inverse(thetahat);
  EXPECT(assert_equal(expectedJacobian, actualJacobian));
}

/* ************************************************************************* */
TEST(Rot3, manifold_caley)
{
  Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCoordinates(gR2, Rot3::CAYLEY);
  CHECK(assert_equal(gR2, gR1.retract(d12, Rot3::CAYLEY)));
  Vector d21 = gR2.localCoordinates(gR1, Rot3::CAYLEY);
  CHECK(assert_equal(gR1, gR2.retract(d21, Rot3::CAYLEY)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = (Vector(3) << 0.1, 0.2, 0.3);
  // exp(-d)=inverse(exp(d))
  CHECK(assert_equal(Rot3::Expmap(-d),Rot3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  Rot3 R2 = Rot3::Expmap (2 * d);
  Rot3 R3 = Rot3::Expmap (3 * d);
  Rot3 R5 = Rot3::Expmap (5 * d);
  CHECK(assert_equal(R5,R2*R3));
  CHECK(assert_equal(R5,R3*R2));
}

/* ************************************************************************* */
TEST(Rot3, manifold_slow_caley)
{
  Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCoordinates(gR2, Rot3::SLOW_CAYLEY);
  CHECK(assert_equal(gR2, gR1.retract(d12, Rot3::SLOW_CAYLEY)));
  Vector d21 = gR2.localCoordinates(gR1, Rot3::SLOW_CAYLEY);
  CHECK(assert_equal(gR1, gR2.retract(d21, Rot3::SLOW_CAYLEY)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = (Vector(3) << 0.1, 0.2, 0.3);
  // exp(-d)=inverse(exp(d))
  CHECK(assert_equal(Rot3::Expmap(-d),Rot3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  Rot3 R2 = Rot3::Expmap (2 * d);
  Rot3 R3 = Rot3::Expmap (3 * d);
  Rot3 R5 = Rot3::Expmap (5 * d);
  CHECK(assert_equal(R5,R2*R3));
  CHECK(assert_equal(R5,R3*R2));
}

/* ************************************************************************* */
TEST(Rot3, manifold_expmap)
{
  Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
  Rot3 origin;

  // log behaves correctly
  Vector d12 = gR1.localCoordinates(gR2, Rot3::EXPMAP);
  CHECK(assert_equal(gR2, gR1.retract(d12, Rot3::EXPMAP)));
  Vector d21 = gR2.localCoordinates(gR1, Rot3::EXPMAP);
  CHECK(assert_equal(gR1, gR2.retract(d21, Rot3::EXPMAP)));

  // Check that it is expmap
  CHECK(assert_equal(gR2, gR1*Rot3::Expmap(d12)));
  CHECK(assert_equal(gR1, gR2*Rot3::Expmap(d21)));

  // Check that log(t1,t2)=-log(t2,t1)
  CHECK(assert_equal(d12,-d21));

  // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
  Vector d = (Vector(3) << 0.1, 0.2, 0.3);
  // exp(-d)=inverse(exp(d))
  CHECK(assert_equal(Rot3::Expmap(-d),Rot3::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  Rot3 R2 = Rot3::Expmap (2 * d);
  Rot3 R3 = Rot3::Expmap (3 * d);
  Rot3 R5 = Rot3::Expmap (5 * d);
  CHECK(assert_equal(R5,R2*R3));
  CHECK(assert_equal(R5,R3*R2));
}

/* ************************************************************************* */
class AngularVelocity: public Point3 {
public:
  AngularVelocity(const Point3& p) :
    Point3(p) {
  }
  AngularVelocity(double wx, double wy, double wz) :
    Point3(wx, wy, wz) {
  }
};

AngularVelocity bracket(const AngularVelocity& X, const AngularVelocity& Y) {
  return X.cross(Y);
}

/* ************************************************************************* */
TEST(Rot3, BCH)
{
  // Approximate exmap by BCH formula
  AngularVelocity w1(0.2, -0.1, 0.1);
  AngularVelocity w2(0.01, 0.02, -0.03);
  Rot3 R1 = Rot3::Expmap (w1.vector()), R2 = Rot3::Expmap (w2.vector());
  Rot3 R3 = R1 * R2;
  Vector expected = Rot3::Logmap(R3);
  Vector actual = BCH(w1, w2).vector();
  CHECK(assert_equal(expected, actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, rotate_derivatives)
{
  Matrix actualDrotate1a, actualDrotate1b, actualDrotate2;
  R.rotate(P, actualDrotate1a, actualDrotate2);
  R.inverse().rotate(P, actualDrotate1b, boost::none);
  Matrix numerical1 = numericalDerivative21(testing::rotate<Rot3,Point3>, R, P);
  Matrix numerical2 = numericalDerivative21(testing::rotate<Rot3,Point3>, R.inverse(), P);
  Matrix numerical3 = numericalDerivative22(testing::rotate<Rot3,Point3>, R, P);
  EXPECT(assert_equal(numerical1,actualDrotate1a,error));
  EXPECT(assert_equal(numerical2,actualDrotate1b,error));
  EXPECT(assert_equal(numerical3,actualDrotate2, error));
}

/* ************************************************************************* */
TEST( Rot3, unrotate)
{
  Point3 w = R * P;
  Matrix H1,H2;
  Point3 actual = R.unrotate(w,H1,H2);
  CHECK(assert_equal(P,actual));

  Matrix numerical1 = numericalDerivative21(testing::unrotate<Rot3,Point3>, R, w);
  CHECK(assert_equal(numerical1,H1,error));

  Matrix numerical2 = numericalDerivative22(testing::unrotate<Rot3,Point3>, R, w);
  CHECK(assert_equal(numerical2,H2,error));
}

/* ************************************************************************* */
TEST( Rot3, compose )
{
  Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::rodriguez(0.2, 0.3, 0.5);

  Rot3 expected = R1 * R2;
  Matrix actualH1, actualH2;
  Rot3 actual = R1.compose(R2, actualH1, actualH2);
  CHECK(assert_equal(expected,actual));

  Matrix numericalH1 = numericalDerivative21(testing::compose<Rot3>, R1,
      R2, 1e-2);
  CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(testing::compose<Rot3>, R1,
      R2, 1e-2);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Rot3, inverse )
{
  Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);

  Rot3 I;
  Matrix actualH;
  CHECK(assert_equal(I,R*R.inverse(actualH)));
  CHECK(assert_equal(I,R.inverse()*R));

  Matrix numericalH = numericalDerivative11(testing::inverse<Rot3>, R);
  CHECK(assert_equal(numericalH,actualH));
}

/* ************************************************************************* */
TEST( Rot3, between )
{
  Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);
  Rot3 origin;
  CHECK(assert_equal(R, origin.between(R)));
  CHECK(assert_equal(R.inverse(), R.between(origin)));

  Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::rodriguez(0.2, 0.3, 0.5);

  Rot3 expected = R1.inverse() * R2;
  Matrix actualH1, actualH2;
  Rot3 actual = R1.between(R2, actualH1, actualH2);
  CHECK(assert_equal(expected,actual));

  Matrix numericalH1 = numericalDerivative21(testing::between<Rot3> , R1, R2);
  CHECK(assert_equal(numericalH1,actualH1));

  Matrix numericalH2 = numericalDerivative22(testing::between<Rot3> , R1, R2);
  CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
Vector w = (Vector(3) << 0.1, 0.27, -0.2);

// Left trivialization Derivative of exp(w) over w: How exp(w) changes when w changes?
// We find y such that: exp(w) exp(y) = exp(w + dw) for dw --> 0
// => y = log (exp(-w) * exp(w+dw))
Vector testDexpL(const LieVector& dw) {
  Vector y = Rot3::Logmap(Rot3::Expmap(-w) * Rot3::Expmap(w + dw));
  return y;
}

TEST( Rot3, dexpL) {
  Matrix actualDexpL = Rot3::dexpL(w);
  Matrix expectedDexpL = numericalDerivative11(
      boost::function<Vector(const LieVector&)>(
          boost::bind(testDexpL, _1)), LieVector(zero(3)), 1e-2);
  EXPECT(assert_equal(expectedDexpL, actualDexpL, 1e-5));

  Matrix actualDexpInvL = Rot3::dexpInvL(w);
  EXPECT(assert_equal(expectedDexpL.inverse(), actualDexpInvL, 1e-5));
}

/* ************************************************************************* */
TEST( Rot3, xyz )
{
  double t = 0.1, st = sin(t), ct = cos(t);

  // Make sure all counterclockwise
  // Diagrams below are all from from unchanging axis

  // z
  // |   * Y=(ct,st)
  // x----y
  Rot3 expected1(1, 0, 0, 0, ct, -st, 0, st, ct);
  CHECK(assert_equal(expected1,Rot3::Rx(t)));

  // x
  // |   * Z=(ct,st)
  // y----z
  Rot3 expected2(ct, 0, st, 0, 1, 0, -st, 0, ct);
  CHECK(assert_equal(expected2,Rot3::Ry(t)));

  // y
  // |   X=* (ct,st)
  // z----x
  Rot3 expected3(ct, -st, 0, st, ct, 0, 0, 0, 1);
  CHECK(assert_equal(expected3,Rot3::Rz(t)));

  // Check compound rotation
  Rot3 expected = Rot3::Rz(0.3) * Rot3::Ry(0.2) * Rot3::Rx(0.1);
  CHECK(assert_equal(expected,Rot3::RzRyRx(0.1,0.2,0.3)));
}

/* ************************************************************************* */
TEST( Rot3, yaw_pitch_roll )
{
  double t = 0.1;

  // yaw is around z axis
  CHECK(assert_equal(Rot3::Rz(t),Rot3::yaw(t)));

  // pitch is around y axis
  CHECK(assert_equal(Rot3::Ry(t),Rot3::pitch(t)));

  // roll is around x axis
  CHECK(assert_equal(Rot3::Rx(t),Rot3::roll(t)));

  // Check compound rotation
  Rot3 expected = Rot3::yaw(0.1) * Rot3::pitch(0.2) * Rot3::roll(0.3);
  CHECK(assert_equal(expected,Rot3::ypr(0.1,0.2,0.3)));

  CHECK(assert_equal((Vector)(Vector(3) << 0.1, 0.2, 0.3),expected.ypr()));
}

/* ************************************************************************* */
TEST( Rot3, RQ)
{
  // Try RQ on a pure rotation
  Matrix actualK;
  Vector actual;
  boost::tie(actualK, actual) = RQ(R.matrix());
  Vector expected = (Vector(3) << 0.14715, 0.385821, 0.231671);
  CHECK(assert_equal(I3,actualK));
  CHECK(assert_equal(expected,actual,1e-6));

  // Try using xyz call, asserting that Rot3::RzRyRx(x,y,z).xyz()==[x;y;z]
  CHECK(assert_equal(expected,R.xyz(),1e-6));
  CHECK(assert_equal((Vector)(Vector(3) << 0.1,0.2,0.3),Rot3::RzRyRx(0.1,0.2,0.3).xyz()));

  // Try using ypr call, asserting that Rot3::ypr(y,p,r).ypr()==[y;p;r]
  CHECK(assert_equal((Vector)(Vector(3) << 0.1,0.2,0.3),Rot3::ypr(0.1,0.2,0.3).ypr()));
  CHECK(assert_equal((Vector)(Vector(3) << 0.3,0.2,0.1),Rot3::ypr(0.1,0.2,0.3).rpy()));

  // Try ypr for pure yaw-pitch-roll matrices
  CHECK(assert_equal((Vector)(Vector(3) << 0.1,0.0,0.0),Rot3::yaw (0.1).ypr()));
  CHECK(assert_equal((Vector)(Vector(3) << 0.0,0.1,0.0),Rot3::pitch(0.1).ypr()));
  CHECK(assert_equal((Vector)(Vector(3) << 0.0,0.0,0.1),Rot3::roll (0.1).ypr()));

  // Try RQ to recover calibration from 3*3 sub-block of projection matrix
  Matrix K = (Matrix(3, 3) << 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
  Matrix A = K * R.matrix();
  boost::tie(actualK, actual) = RQ(A);
  CHECK(assert_equal(K,actualK));
  CHECK(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
TEST( Rot3, expmapStability ) {
  Vector w = (Vector(3) << 78e-9, 5e-8, 97e-7);
  double theta = w.norm();
  double theta2 = theta*theta;
  Rot3 actualR = Rot3::Expmap(w);
  Matrix W = (Matrix(3, 3) << 0.0, -w(2), w(1),
                          w(2), 0.0, -w(0),
                          -w(1), w(0), 0.0 );
  Matrix W2 = W*W;
  Matrix Rmat = I3 + (1.0-theta2/6.0 + theta2*theta2/120.0
      - theta2*theta2*theta2/5040.0)*W + (0.5 - theta2/24.0 + theta2*theta2/720.0)*W2 ;
  Rot3 expectedR( Rmat );
  CHECK(assert_equal(expectedR, actualR, 1e-10));
}

/* ************************************************************************* */
TEST( Rot3, logmapStability ) {
  Vector w = (Vector(3) << 1e-8, 0.0, 0.0);
  Rot3 R = Rot3::Expmap(w);
//  double tr = R.r1().x()+R.r2().y()+R.r3().z();
//  std::cout.precision(5000);
//  std::cout << "theta: " << w.norm() << std::endl;
//  std::cout << "trace: " << tr << std::endl;
//  R.print("R = ");
  Vector actualw = Rot3::Logmap(R);
  CHECK(assert_equal(w, actualw, 1e-15));
}

/* ************************************************************************* */
TEST(Rot3, quaternion) {
  // NOTE: This is also verifying the ability to convert Vector to Quaternion
  Quaternion q1(0.710997408193224, 0.360544029310185, 0.594459869568306, 0.105395217842782);
  Rot3 R1 = Rot3((Matrix)(Matrix(3, 3) <<
      0.271018623057411,   0.278786459830371,   0.921318086098018,
      0.578529366719085,   0.717799701969298,  -0.387385285854279,
     -0.769319620053772,   0.637998195662053,   0.033250932803219));

  Quaternion q2(0.263360579192421, 0.571813128030932, 0.494678363680335, 0.599136268678053);
  Rot3 R2 = Rot3((Matrix)(Matrix(3, 3) <<
      -0.207341903877828,   0.250149415542075,   0.945745528564780,
       0.881304914479026,  -0.371869043667957,   0.291573424846290,
       0.424630407073532,   0.893945571198514,  -0.143353873763946));

  // Check creating Rot3 from quaternion
  EXPECT(assert_equal(R1, Rot3(q1)));
  EXPECT(assert_equal(R1, Rot3::quaternion(q1.w(), q1.x(), q1.y(), q1.z())));
  EXPECT(assert_equal(R2, Rot3(q2)));
  EXPECT(assert_equal(R2, Rot3::quaternion(q2.w(), q2.x(), q2.y(), q2.z())));

  // Check converting Rot3 to quaterion
  EXPECT(assert_equal(Vector(R1.toQuaternion().coeffs()), Vector(q1.coeffs())));
  EXPECT(assert_equal(Vector(R2.toQuaternion().coeffs()), Vector(q2.coeffs())));

  // Check that quaternion and Rot3 represent the same rotation
  Point3 p1(1.0, 2.0, 3.0);
  Point3 p2(8.0, 7.0, 9.0);

  Point3 expected1 = R1*p1;
  Point3 expected2 = R2*p2;

  Point3 actual1 = Point3(q1*p1.vector());
  Point3 actual2 = Point3(q2*p2.vector());

  EXPECT(assert_equal(expected1, actual1));
  EXPECT(assert_equal(expected2, actual2));
}

/* ************************************************************************* */
TEST( Rot3, Cayley ) {
  Matrix A = skewSymmetric(1,2,-3);
  Matrix Q = Cayley(A);
  EXPECT(assert_equal(I3, trans(Q)*Q));
  EXPECT(assert_equal(A, Cayley(Q)));
}

/* ************************************************************************* */
TEST( Rot3, stream)
{
  Rot3 R;
  std::ostringstream os;
  os << R;
  EXPECT(os.str() == "\n|1, 0, 0|\n|0, 1, 0|\n|0, 0, 1|\n");
}

#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

