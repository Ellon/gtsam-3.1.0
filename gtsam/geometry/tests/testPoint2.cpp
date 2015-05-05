/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPoint2.cpp
 * @brief  Unit tests for Point2 class
 * @author Frank Dellaert
 **/

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point2)
GTSAM_CONCEPT_LIE_INST(Point2)

/* ************************************************************************* */
TEST(Point2, constructor) {
  Point2 p1(1, 2), p2 = p1;
  EXPECT(assert_equal(p1, p2));
}

/* ************************************************************************* */
TEST(Point2, Lie) {
  Point2 p1(1, 2), p2(4, 5);
  Matrix H1, H2;

  EXPECT(assert_equal(Point2(5,7), p1.compose(p2, H1, H2)));
  EXPECT(assert_equal(eye(2), H1));
  EXPECT(assert_equal(eye(2), H2));

  EXPECT(assert_equal(Point2(3,3), p1.between(p2, H1, H2)));
  EXPECT(assert_equal(-eye(2), H1));
  EXPECT(assert_equal(eye(2), H2));

  EXPECT(assert_equal(Point2(5,7), p1.retract((Vector(2) << 4., 5.))));
  EXPECT(assert_equal((Vector(2) << 3.,3.), p1.localCoordinates(p2)));
}

/* ************************************************************************* */
TEST( Point2, expmap) {
  Vector d(2);
  d(0) = 1;
  d(1) = -1;
  Point2 a(4, 5), b = a.retract(d), c(5, 4);
  EXPECT(assert_equal(b,c));
}

/* ************************************************************************* */
TEST( Point2, arithmetic) {
  EXPECT(assert_equal( Point2(-5,-6), -Point2(5,6) ));
  EXPECT(assert_equal( Point2(5,6), Point2(4,5)+Point2(1,1)));
  EXPECT(assert_equal( Point2(3,4), Point2(4,5)-Point2(1,1) ));
  EXPECT(assert_equal( Point2(8,6), Point2(4,3)*2));
  EXPECT(assert_equal( Point2(4,6), 2*Point2(2,3)));
  EXPECT(assert_equal( Point2(2,3), Point2(4,6)/2));
}

/* ************************************************************************* */
TEST( Point2, unit) {
  Point2 p0(10, 0), p1(0, -10), p2(10, 10);
  EXPECT(assert_equal(Point2(1, 0), p0.unit(), 1e-6));
  EXPECT(assert_equal(Point2(0,-1), p1.unit(), 1e-6));
  EXPECT(assert_equal(Point2(sqrt(2.0)/2.0, sqrt(2.0)/2.0), p2.unit(), 1e-6));
}

namespace {
  /* ************************************************************************* */
  // some shared test values
  Point2 x1, x2(1, 1), x3(1, 1);
  Point2 l1(1, 0), l2(1, 1), l3(2, 2), l4(1, 3);

  /* ************************************************************************* */
  LieVector norm_proxy(const Point2& point) {
    return LieVector(point.norm());
  }
}
TEST( Point2, norm ) {
  Point2 p0(cos(5.0), sin(5.0));
  DOUBLES_EQUAL(1, p0.norm(), 1e-6);
  Point2 p1(4, 5), p2(1, 1);
  DOUBLES_EQUAL( 5, p1.distance(p2), 1e-6);
  DOUBLES_EQUAL( 5, (p2-p1).norm(), 1e-6);

  Matrix expectedH, actualH;
  double actual;

  // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
  actual = x1.norm(actualH);
  EXPECT_DOUBLES_EQUAL(0, actual, 1e-9);
  expectedH = (Matrix(1, 2) << 1.0, 1.0);
  EXPECT(assert_equal(expectedH,actualH));

  actual = x2.norm(actualH);
  EXPECT_DOUBLES_EQUAL(sqrt(2.0), actual, 1e-9);
  expectedH = numericalDerivative11(norm_proxy, x2);
  EXPECT(assert_equal(expectedH,actualH));
}

/* ************************************************************************* */
namespace {
  LieVector distance_proxy(const Point2& location, const Point2& point) {
    return LieVector(location.distance(point));
  }
}
TEST( Point2, distance ) {
  Matrix expectedH1, actualH1, expectedH2, actualH2;

  // establish distance is indeed zero
  EXPECT_DOUBLES_EQUAL(1, x1.distance(l1), 1e-9);

  // establish distance is indeed 45 degrees
  EXPECT_DOUBLES_EQUAL(sqrt(2.0), x1.distance(l2), 1e-9);

  // Another pair
  double actual23 = x2.distance(l3, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(sqrt(2.0), actual23, 1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(distance_proxy, x2, l3);
  expectedH2 = numericalDerivative22(distance_proxy, x2, l3);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));

  // Another test
  double actual34 = x3.distance(l4, actualH1, actualH2);
  EXPECT_DOUBLES_EQUAL(2, actual34, 1e-9);

  // Check numerical derivatives
  expectedH1 = numericalDerivative21(distance_proxy, x3, l4);
  expectedH2 = numericalDerivative22(distance_proxy, x3, l4);
  EXPECT(assert_equal(expectedH1,actualH1));
  EXPECT(assert_equal(expectedH2,actualH2));
}

/* ************************************************************************* */
TEST( Point2, circleCircleIntersection) {

  double offset = 0.994987;
  // Test intersections of circle moving from inside to outside

  list<Point2> inside = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(0,0),1);
  EXPECT_LONGS_EQUAL(0,inside.size());

  list<Point2> touching1 = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(4,0),1);
  EXPECT_LONGS_EQUAL(1,touching1.size());
  EXPECT(assert_equal(Point2(5,0), touching1.front()));

  list<Point2> common = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(5,0),1);
  EXPECT_LONGS_EQUAL(2,common.size());
  EXPECT(assert_equal(Point2(4.9,  offset), common.front(), 1e-6));
  EXPECT(assert_equal(Point2(4.9, -offset), common.back(), 1e-6));

  list<Point2> touching2 = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(6,0),1);
  EXPECT_LONGS_EQUAL(1,touching2.size());
  EXPECT(assert_equal(Point2(5,0), touching2.front()));

  // test rotated case
  list<Point2> rotated = Point2::CircleCircleIntersection(Point2(0,0),5,Point2(0,5),1);
  EXPECT_LONGS_EQUAL(2,rotated.size());
  EXPECT(assert_equal(Point2(-offset, 4.9), rotated.front(), 1e-6));
  EXPECT(assert_equal(Point2( offset, 4.9), rotated.back(), 1e-6));

  // test r1<r2
  list<Point2> smaller = Point2::CircleCircleIntersection(Point2(0,0),1,Point2(5,0),5);
  EXPECT_LONGS_EQUAL(2,smaller.size());
  EXPECT(assert_equal(Point2(0.1,  offset), smaller.front(), 1e-6));
  EXPECT(assert_equal(Point2(0.1, -offset), smaller.back(), 1e-6));

  // test offset case, r1>r2
  list<Point2> offset1 = Point2::CircleCircleIntersection(Point2(1,1),5,Point2(6,1),1);
  EXPECT_LONGS_EQUAL(2,offset1.size());
  EXPECT(assert_equal(Point2(5.9, 1+offset), offset1.front(), 1e-6));
  EXPECT(assert_equal(Point2(5.9, 1-offset), offset1.back(), 1e-6));

  // test offset case, r1<r2
  list<Point2> offset2 = Point2::CircleCircleIntersection(Point2(6,1),1,Point2(1,1),5);
  EXPECT_LONGS_EQUAL(2,offset2.size());
  EXPECT(assert_equal(Point2(5.9, 1-offset), offset2.front(), 1e-6));
  EXPECT(assert_equal(Point2(5.9, 1+offset), offset2.back(), 1e-6));

}

/* ************************************************************************* */
TEST( Point2, stream) {
  Point2 p(1, 2);
  std::ostringstream os;
  os << p;
  EXPECT(os.str() == "(1, 2)");
}

/* ************************************************************************* */
int main () {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

