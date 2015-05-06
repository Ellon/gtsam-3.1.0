/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testParallaxAnglePoint3.cpp
 * @brief  Unit tests for ParallaxAnglePoint3 class
 * @author Ellon P. Mendes
 **/

#include <gtsam/geometry/ParallaxAnglePoint3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(ParallaxAnglePoint3)
GTSAM_CONCEPT_MANIFOLD_INST(ParallaxAnglePoint3)

/* ************************************************************************* */
TEST(ParallaxAnglePoint3, constructor) {
  ParallaxAnglePoint3 p1(1, 2, 3), p2 = p1;
  EXPECT(assert_equal(p1, p2));
}

/* ************************************************************************* */
// TEST(ParallaxAnglePoint2, Lie) {
//   Point2 p1(1, 2), p2(4, 5);
//   Matrix H1, H2;

//   EXPECT(assert_equal(Point2(5,7), p1.compose(p2, H1, H2)));
//   EXPECT(assert_equal(eye(2), H1));
//   EXPECT(assert_equal(eye(2), H2));

//   EXPECT(assert_equal(Point2(3,3), p1.between(p2, H1, H2)));
//   EXPECT(assert_equal(-eye(2), H1));
//   EXPECT(assert_equal(eye(2), H2));

//   EXPECT(assert_equal(Point2(5,7), p1.retract((Vector(2) << 4., 5.))));
//   EXPECT(assert_equal((Vector(2) << 3.,3.), p1.localCoordinates(p2)));
// }

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, expmap) {
  Vector d(3);
  d(0) =  1.;
  d(1) = -1.;
  d(2) =  2.;
  ParallaxAnglePoint3 a(4., 5., 6.), b = a.retract(d), c(5., 4., 8.);
  EXPECT(assert_equal(c,b));
  EXPECT(assert_equal(d,a.localCoordinates(c)));
}

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, arithmetic) {
  // EXPECT(assert_equal( ParallaxAnglePoint2(-5,-6), -ParallaxAnglePoint2(5,6) ));
  EXPECT(assert_equal( ParallaxAnglePoint3(5,6,7), ParallaxAnglePoint3(4,5,6)+ParallaxAnglePoint3(1,1,1)));
  EXPECT(assert_equal( ParallaxAnglePoint3(3,4,5), ParallaxAnglePoint3(4,5,6)-ParallaxAnglePoint3(1,1,1)));
  // EXPECT(assert_equal( ParallaxAnglePoint2(8,6), ParallaxAnglePoint2(4,3)*2));
  // EXPECT(assert_equal( ParallaxAnglePoint2(4,6), 2*ParallaxAnglePoint2(2,3)));
  // EXPECT(assert_equal( ParallaxAnglePoint2(2,3), ParallaxAnglePoint2(4,6)/2));
}

// namespace {
//   /* ************************************************************************* */
//   // some shared test values
//   ParallaxAnglePoint2 x1, x2(1, 1), x3(1, 1);
//   ParallaxAnglePoint2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

//   /* ************************************************************************* */
//   // LieVector norm_proxy(const ParallaxAnglePoint2& point) {
//   //   return LieVector(point.norm());
//   // }
// }
// TEST( ParallaxAnglePoint2, norm ) {
//   Point2 p0(cos(5.0), sin(5.0));
//   DOUBLES_EQUAL(1, p0.norm(), 1e-6);
//   Point2 p1(4, 5), p2(1, 1);
//   DOUBLES_EQUAL( 5, p1.distance(p2), 1e-6);
//   DOUBLES_EQUAL( 5, (p2-p1).norm(), 1e-6);

//   Matrix expectedH, actualH;
//   double actual;

//   // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
//   actual = x1.norm(actualH);
//   EXPECT_DOUBLES_EQUAL(0, actual, 1e-9);
//   expectedH = (Matrix(1, 2) << 1.0, 1.0);
//   EXPECT(assert_equal(expectedH,actualH));

//   actual = x2.norm(actualH);
//   EXPECT_DOUBLES_EQUAL(sqrt(2.0), actual, 1e-9);
//   expectedH = numericalDerivative11(norm_proxy, x2);
//   EXPECT(assert_equal(expectedH,actualH));
// }

/* ************************************************************************* */
// namespace {
//   LieVector distance_proxy(const Point2& location, const Point2& point) {
//     return LieVector(location.distance(point));
//   }
// }
// TEST( Point2, distance ) {
//   Matrix expectedH1, actualH1, expectedH2, actualH2;

//   // establish distance is indeed zero
//   EXPECT_DOUBLES_EQUAL(1, x1.distance(l1), 1e-9);

//   // establish distance is indeed 45 degrees
//   EXPECT_DOUBLES_EQUAL(sqrt(2.0), x1.distance(l2), 1e-9);

//   // Another pair
//   double actual23 = x2.distance(l3, actualH1, actualH2);
//   EXPECT_DOUBLES_EQUAL(sqrt(2.0), actual23, 1e-9);

//   // Check numerical derivatives
//   expectedH1 = numericalDerivative21(distance_proxy, x2, l3);
//   expectedH2 = numericalDerivative22(distance_proxy, x2, l3);
//   EXPECT(assert_equal(expectedH1,actualH1));
//   EXPECT(assert_equal(expectedH2,actualH2));

//   // Another test
//   double actual34 = x3.distance(l4, actualH1, actualH2);
//   EXPECT_DOUBLES_EQUAL(2, actual34, 1e-9);

//   // Check numerical derivatives
//   expectedH1 = numericalDerivative21(distance_proxy, x3, l4);
//   expectedH2 = numericalDerivative22(distance_proxy, x3, l4);
//   EXPECT(assert_equal(expectedH1,actualH1));
//   EXPECT(assert_equal(expectedH2,actualH2));
// }

/* ************************************************************************* */
// TEST( Point2, circleCircleIntersection) {

//   double offset = 0.994987;
//   // Test intersections of circle moving from inside to outside

//   list<Point2> inside = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(0,0),1);
//   EXPECT_LONGS_EQUAL(0,inside.size());

//   list<Point2> touching1 = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(4,0),1);
//   EXPECT_LONGS_EQUAL(1,touching1.size());
//   EXPECT(assert_equal(Point2(5,0), touching1.front()));

//   list<Point2> common = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(5,0),1);
//   EXPECT_LONGS_EQUAL(2,common.size());
//   EXPECT(assert_equal(Point2(4.9,  offset), common.front(), 1e-6));
//   EXPECT(assert_equal(Point2(4.9, -offset), common.back(), 1e-6));

//   list<Point2> touching2 = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(6,0),1);
//   EXPECT_LONGS_EQUAL(1,touching2.size());
//   EXPECT(assert_equal(Point2(5,0), touching2.front()));

//   // test rotated case
//   list<Point2> rotated = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(0,5),1);
//   EXPECT_LONGS_EQUAL(2,rotated.size());
//   EXPECT(assert_equal(Point2(-offset, 4.9), rotated.front(), 1e-6));
//   EXPECT(assert_equal(Point2( offset, 4.9), rotated.back(), 1e-6));

//   // test r1<r2
//   list<Point2> smaller = Point2::CircleCircleIntersection(Point2(0,0),1,Point2(5,0),5);
//   EXPECT_LONGS_EQUAL(2,smaller.size());
//   EXPECT(assert_equal(Point2(0.1,  offset), smaller.front(), 1e-6));
//   EXPECT(assert_equal(Point2(0.1, -offset), smaller.back(), 1e-6));

//   // test offset case, r1>r2
//   list<Point2> offset1 = Point2::CircleCircleIntersection(Point2(1,1),5,Point2(6,1),1);
//   EXPECT_LONGS_EQUAL(2,offset1.size());
//   EXPECT(assert_equal(Point2(5.9, 1+offset), offset1.front(), 1e-6));
//   EXPECT(assert_equal(Point2(5.9, 1-offset), offset1.back(), 1e-6));

//   // test offset case, r1<r2
//   list<Point2> offset2 = Point2::CircleCircleIntersection(Point2(6,1),1,Point2(1,1),5);
//   EXPECT_LONGS_EQUAL(2,offset2.size());
//   EXPECT(assert_equal(Point2(5.9, 1-offset), offset2.front(), 1e-6));
//   EXPECT(assert_equal(Point2(5.9, 1+offset), offset2.back(), 1e-6));

// }

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, stream) {
  ParallaxAnglePoint3 p(1, 2, 3);
  std::ostringstream os;
  os << p;
  EXPECT(os.str() == "(1, 2, 3)");
}

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, directionVector) {
  Point3 mainAnchor(0, -3, 0);
  Point3 assoAnchor(0, -5, 5);
  Point3 otheAnchor(7,  0, 0);
  ParallaxAnglePoint3 p(M_PI/2, 0, M_PI/4);

  Vector ExpectedVecFromMain = (Vector3() << 0, 1, 0);
  Vector ExpectedVecFromOthe = (Vector3() << -4.949747468305833, 0, 0);
  Vector ExpectedVecFromAsso  = (Vector3() << 0, 3.535533905932738, -3.535533905932738);

  // Jacobian matrix evaluated with the values above (check ParallaxPointJacobianTest.m)
  Matrix3 EXPECTED_DIRECVECFROMOTHER_tm = (Matrix3() << 0.707106781186548,                   0,                   0,
                                                                        0,  -0.000000000000000,  -0.707106781186548,
                                                                        0,                   0,   0.707106781186548);

  Matrix3 EXPECTED_DIRECVECFROMOTHER_ta = (Matrix3() << 0,                   0,                   0,
                                                        0,   0.707106781186548,   0.707106781186548,
                                                        0,                   0,                   0);

  Matrix3 EXPECTED_DIRECVECFROMOTHER_to = (Matrix3() << -0.707106781186548,                   0,                   0,
                                                                         0,  -0.707106781186548,                   0,
                                                                         0,                   0,  -0.707106781186548);

  Matrix3 EXPECTED_DIRECVECFROMOTHER_point = (Matrix3() << -2.121320343559642,                   0,  -4.949747468305833,
                                                                            0,   4.949747468305833,  -7.071067811865476,
                                                                            0,   2.121320343559642,                   0);

  Matrix3 EXPECTED_DIRECVECFROMASSO_tm = (Matrix3() << 0.707106781186548,                   0,                   0,
                                                                       0,  -0.000000000000000,  -0.707106781186548,
                                                                       0,                   0,   0.707106781186548);

  Matrix3 EXPECTED_DIRECVECFROMASSO_ta = (Matrix3() << -0.707106781186548,                   0,                   0,
                                                                        0,   0.000000000000000,   0.707106781186548,
                                                                        0,                   0,  -0.707106781186548);

  Matrix3 EXPECTED_DIRECVECFROMASSO_point = (Matrix3() << -2.121320343559642,                   0,                   0,
                                                                           0,   4.949747468305833,  -3.535533905932738,
                                                                           0,   2.121320343559642,  -3.535533905932738);

  Matrix3 EXPECTED_DIRECVECFROMMAIN_point = (Matrix3() << -1,     0,     0,
                                                           0,     0,     0,
                                                           0,     1,     0);

  Matrix H_point, H_main, H_asso, H_other;
  CHECK(assert_equal(ExpectedVecFromMain, p.directionVectorFromMainAnchor(), 1e-6));
  CHECK(assert_equal(ExpectedVecFromMain, p.directionVectorFromMainAnchor(H_point), 1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMMAIN_point, H_point, 1e-6));

  H_point = zeros(3,3);
  CHECK(assert_equal(ExpectedVecFromAsso, p.directionVectorFromAssoAnchor(mainAnchor, assoAnchor), 1e-6));
  CHECK(assert_equal(ExpectedVecFromAsso, p.directionVectorFromAssoAnchor(mainAnchor, assoAnchor, H_point, H_main, H_asso), 1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMASSO_point, H_point, 1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMASSO_tm,    H_main,  1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMASSO_ta,    H_asso,  1e-6));


  H_point = zeros(3,3);
  H_main  = zeros(3,3);
  H_asso  = zeros(3,3);
  CHECK(assert_equal(ExpectedVecFromOthe, p.directionVectorFromOtheAnchor(mainAnchor, assoAnchor, otheAnchor), 1e-6));
  CHECK(assert_equal(ExpectedVecFromOthe, p.directionVectorFromOtheAnchor(mainAnchor, assoAnchor, otheAnchor, H_point, H_main, H_asso, H_other), 1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMOTHER_point, H_point,  1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMOTHER_tm,    H_main,   1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMOTHER_ta,    H_asso,   1e-6));
  EXPECT(assert_equal(EXPECTED_DIRECVECFROMOTHER_to,    H_other,  1e-6));

}

/* ************************************************************************* */
int main () {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

