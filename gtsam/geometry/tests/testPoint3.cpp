/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPoint3.cpp
 * @brief  Unit tests for Point3 class
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point3)
GTSAM_CONCEPT_LIE_INST(Point3)

static Point3 P(0.2, 0.7, -2);

/* ************************************************************************* */
TEST(Point3, Lie) {
  Point3 p1(1, 2, 3);
  Point3 p2(4, 5, 6);
  Matrix H1, H2;

  EXPECT(assert_equal(Point3(5, 7, 9), p1.compose(p2, H1, H2)));
  EXPECT(assert_equal(eye(3), H1));
  EXPECT(assert_equal(eye(3), H2));

  EXPECT(assert_equal(Point3(3, 3, 3), p1.between(p2, H1, H2)));
  EXPECT(assert_equal(-eye(3), H1));
  EXPECT(assert_equal(eye(3), H2));

  EXPECT(assert_equal(Point3(5, 7, 9), p1.retract((Vector(3) << 4., 5., 6.))));
  EXPECT(assert_equal((Vector)(Vector(3) << 3.,3.,3.), p1.localCoordinates(p2)));
}

/* ************************************************************************* */
TEST( Point3, arithmetic) {
  CHECK(P * 3 == 3 * P);
  CHECK(assert_equal(Point3(-1, -5, -6), -Point3(1, 5, 6)));
  CHECK(assert_equal(Point3(2, 5, 6), Point3(1, 4, 5) + Point3(1, 1, 1)));
  CHECK(assert_equal(Point3(0, 3, 4), Point3(1, 4, 5) - Point3(1, 1, 1)));
  CHECK(assert_equal(Point3(2, 8, 6), Point3(1, 4, 3) * 2));
  CHECK(assert_equal(Point3(2, 2, 6), 2 * Point3(1, 1, 3)));
  CHECK(assert_equal(Point3(1, 2, 3), Point3(2, 4, 6) / 2));
}

/* ************************************************************************* */
TEST( Point3, equals) {
  CHECK(P.equals(P));
  Point3 Q;
  CHECK(!P.equals(Q));
}

/* ************************************************************************* */
TEST( Point3, dot) {
  Point3 origin, ones(1, 1, 1);
  CHECK(origin.dot(Point3(1, 1, 0)) == 0);
  CHECK(ones.dot(Point3(1, 1, 0)) == 2);
}

/* ************************************************************************* */
TEST( Point3, stream) {
  Point3 p(1, 2, -3);
  std::ostringstream os;
  os << p;
  EXPECT(os.str() == "[1, 2, -3]';");
}

//*************************************************************************
TEST (Point3, normalize) {
  Matrix actualH;
  Point3 point(1, -2, 3); // arbitrary point
  Point3 expected(point / sqrt(14.0));
  EXPECT(assert_equal(expected, point.normalize(actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<Point3, Point3>(
      boost::bind(&Point3::normalize, _1, boost::none), point);
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

