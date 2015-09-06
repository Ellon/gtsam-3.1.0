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

#include <gtsam/geometry/ParallaxAnglePointTools.h>
#include <gtsam/base/Testable.h>
// #include <gtsam/base/numericalDerivative.h>
// #include <gtsam/base/lieProxies.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( ParallaxAnglePointTools, vec2py_and_py2vec) {

  // Data and expected return values
  Vector vec = (Vector3() << 1, 2, 3);
  Matrix eVEC_py = (Matrix(3,2) <<
    -3.585685828003182e-01,    -5.345224838248487e-01,
    -7.171371656006362e-01,     2.672612419124245e-01,
     5.976143046671968e-01,                         0);

  Vector ePy = (Vector2() << 9.302740141154721e-01, 1.107148717794090e+00);
  Matrix ePY_vec = (Matrix(2,3) <<
    -9.583148474999099e-02,    -1.916629694999820e-01,     1.597191412499850e-01,
    -4.000000000000000e-01,     2.000000000000000e-01,                         0);

  // Simple calls
  Vector py = vec2py(vec);
  EXPECT(assert_equal(ePy, py, 1e-6));
  EXPECT(assert_equal(vec.normalized(), py2vec(py), 1e-6));

  // With Jacobians
  Matrix PY_vec;
  py = vec2py(vec,PY_vec);
  EXPECT(assert_equal(ePy, py, 1e-6));
  EXPECT(assert_equal(ePY_vec, PY_vec, 1e-6));

  Matrix VEC_py;
  EXPECT(assert_equal(vec.normalized(), py2vec(py,VEC_py), 1e-6));
  EXPECT(assert_equal(eVEC_py, VEC_py, 1e-6));

}

/* ************************************************************************* */
TEST( ParallaxAnglePointTools, norm) {

  Vector vec = (Vector3() << 1, 2, 3);

  // simple call
  EXPECT_DOUBLES_EQUAL(vec.norm(), norm(vec), 1e-6)

  // with jacobian
  Matrix NORM_vec;
  EXPECT_DOUBLES_EQUAL(vec.norm(), norm(vec,NORM_vec), 1e-6);
  EXPECT(assert_equal((vec/vec.norm()).transpose(), NORM_vec, 1e-6));

}

/* ************************************************************************* */
TEST( ParallaxAnglePointTools, dot) {

  Vector v1 = (Vector3() << 1, 2, 3);
  Vector v2 = (Vector3() << 4, 5, 6);

  // simple call
  EXPECT_DOUBLES_EQUAL(v1.dot(v2), dot(v1, v2), 1e-6);

  // with jacobian
  Matrix DOT_v1, DOT_v2;
  EXPECT_DOUBLES_EQUAL(v1.dot(v2), dot(v1, v2, DOT_v1, DOT_v2), 1e-6);
  EXPECT(assert_equal(v2.transpose(), DOT_v1, 1e-6));
  EXPECT(assert_equal(v1.transpose(), DOT_v2, 1e-6));

}

#define radians(deg)  ((deg) * M_PI / 180.0)
#define degrees(rad)  ((rad) * 180.0 / M_PI)

/* ************************************************************************* */
TEST( ParallaxAnglePointTools, vectors2angle) {

  Vector v1 = (Vector3() << 1, 1, 1);
  Vector v2 = (Vector3() << 2, 2, 0);
  Vector v3 = (Vector3() << 0, 0, 3);
  Vector v4 = (Vector3() << -4, -4, -4);
  Vector v5 = (Vector3() << 3.535533905932738e+00, 3.535533905932737e+00, 5.000000000000000e+00);

  // simple calls
  EXPECT_DOUBLES_EQUAL(radians(0.0),   vectors2angle(v1,v1), 1e-6);
  EXPECT_DOUBLES_EQUAL(radians(90.0),  vectors2angle(v2,v3), 1e-6);
  EXPECT_DOUBLES_EQUAL(radians(180.0), vectors2angle(v1,v4), 1e-6);
  EXPECT_DOUBLES_EQUAL(radians(45.0),  vectors2angle(v2,v5), 1e-6);

  // with jacobians
  // v1,v1
  Matrix eANG_arg1 = (Matrix(1,3) << 0, 0, 0);
  Matrix eANG_arg2 = (Matrix(1,3) << 0, 0, 0);
  Matrix ANG_arg1, ANG_arg2;
  EXPECT_DOUBLES_EQUAL(radians(0.0),   vectors2angle(v1, v1, ANG_arg1, ANG_arg2), 1e-6);
  EXPECT(assert_equal(eANG_arg1, ANG_arg1, 1e-6));
  EXPECT(assert_equal(eANG_arg2, ANG_arg2, 1e-6));

  // v2,v3
  eANG_arg1 = (Matrix(1,3) << 0, 0, -3.535533905932738e-01);
  eANG_arg2 = (Matrix(1,3) << -2.357022603955158e-01,    -2.357022603955158e-01, 0);
  EXPECT_DOUBLES_EQUAL(radians(90.0),   vectors2angle(v2, v3, ANG_arg1, ANG_arg2), 1e-6);
  EXPECT(assert_equal(eANG_arg1, ANG_arg1, 1e-6));
  EXPECT(assert_equal(eANG_arg2, ANG_arg2, 1e-6));

  // v1,v4
  eANG_arg1 = (Matrix(1,3) << 0, 0, 0);
  eANG_arg2 = (Matrix(1,3) << 0, 0, 0);
  EXPECT_DOUBLES_EQUAL(radians(180.0),   vectors2angle(v1, v4, ANG_arg1, ANG_arg2), 1e-6);
  EXPECT(assert_equal(eANG_arg1, ANG_arg1, 1e-6));
  EXPECT(assert_equal(eANG_arg2, ANG_arg2, 1e-6));

  // v2,v5
  eANG_arg1 = (Matrix(1,3) << -4.710277376051326e-17,     4.710277376051326e-17,    -3.535533905932738e-01);
  eANG_arg2 = (Matrix(1,3) << -7.071067811865472e-02,    -7.071067811865477e-02,     1.000000000000000e-01);
  EXPECT_DOUBLES_EQUAL(radians(45.0),   vectors2angle(v2, v5, ANG_arg1, ANG_arg2), 1e-6);
  EXPECT(assert_equal(eANG_arg1, ANG_arg1, 1e-6));
  EXPECT(assert_equal(eANG_arg2, ANG_arg2, 1e-6));

}

/* ************************************************************************* */
int main () {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

