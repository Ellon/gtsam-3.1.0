/*
 * @file testRotateFactor.cpp
 * @brief Test RotateFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/slam/RotateFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>
#include <vector>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

//*************************************************************************
// Create some test data
// Let's assume IMU is aligned with aero (X-forward,Z down)
// And camera is looking forward.
Point3 cameraX(0, 1, 0), cameraY(0, 0, 1), cameraZ(1, 0, 0);
Rot3 iRc(cameraX, cameraY, cameraZ);

// Now, let's create some rotations around IMU frame
Unit3 p1(1, 0, 0), p2(0, 1, 0), p3(0, 0, 1);
Rot3 i1Ri2 = Rot3::rodriguez(p1, 1), //
i2Ri3 = Rot3::rodriguez(p2, 1), //
i3Ri4 = Rot3::rodriguez(p3, 1);

// The corresponding rotations in the camera frame
Rot3 c1Zc2 = iRc.inverse() * i1Ri2 * iRc, //
c2Zc3 = iRc.inverse() * i2Ri3 * iRc, //
c3Zc4 = iRc.inverse() * i3Ri4 * iRc;

// The corresponding rotated directions in the camera frame
Unit3 z1 = iRc.inverse() * p1, //
z2 = iRc.inverse() * p2, //
z3 = iRc.inverse() * p3;

typedef noiseModel::Isotropic::shared_ptr Model;

//*************************************************************************
TEST (RotateFactor, checkMath) {
  EXPECT(assert_equal(c1Zc2, Rot3::rodriguez(z1, 1)));
  EXPECT(assert_equal(c2Zc3, Rot3::rodriguez(z2, 1)));
  EXPECT(assert_equal(c3Zc4, Rot3::rodriguez(z3, 1)));
}

//*************************************************************************
TEST (RotateFactor, test) {
  Model model = noiseModel::Isotropic::Sigma(3, 0.01);
  RotateFactor f(1, i1Ri2, c1Zc2, model);
  EXPECT(assert_equal(zero(3), f.evaluateError(iRc), 1e-8));

  Rot3 R = iRc.retract((Vector(3) << 0.1, 0.2, 0.1));
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  Vector expectedE = (Vector(3) << -0.0248752, 0.202981, -0.0890529);
#else
  Vector expectedE = (Vector(3) << -0.0246305, 0.20197, -0.08867);
#endif
  EXPECT( assert_equal(expectedE, f.evaluateError(R), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Rot3>(
        boost::bind(&RotateFactor::evaluateError, &f, _1, boost::none), iRc);
    f.evaluateError(iRc, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Rot3>(
        boost::bind(&RotateFactor::evaluateError, &f, _1, boost::none), R);
    f.evaluateError(R, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
}

//*************************************************************************
TEST (RotateFactor, minimization) {
  // Let's try to recover the correct iRc by minimizing
  NonlinearFactorGraph graph;
  Model model = noiseModel::Isotropic::Sigma(3, 0.01);
  graph.add(RotateFactor(1, i1Ri2, c1Zc2, model));
  graph.add(RotateFactor(1, i2Ri3, c2Zc3, model));
  graph.add(RotateFactor(1, i3Ri4, c3Zc4, model));

  // Check error at ground truth
  Values truth;
  truth.insert(1, iRc);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  double degree = M_PI / 180;
  Rot3 initialE = iRc.retract(degree * (Vector(3) << 20, -20, 20));
  initial.insert(1, initialE);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(3545.40, graph.error(initial), 1);
#else
  EXPECT_DOUBLES_EQUAL(3349, graph.error(initial), 1);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  //parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  Rot3 actual = result.at<Rot3>(1);
  EXPECT(assert_equal(iRc, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

//*************************************************************************
TEST (RotateDirectionsFactor, test) {
  Model model = noiseModel::Isotropic::Sigma(2, 0.01);
  RotateDirectionsFactor f(1, p1, z1, model);
  EXPECT(assert_equal(zero(2), f.evaluateError(iRc), 1e-8));

  Rot3 R = iRc.retract((Vector(3) << 0.1, 0.2, 0.1));

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  Vector expectedE = (Vector(2) << -0.0890529, -0.202981);
#else
  Vector expectedE = (Vector(2) << -0.08867, -0.20197);
#endif

  EXPECT( assert_equal(expectedE, f.evaluateError(R), 1e-5));

  Matrix actual, expected;
  // Use numerical derivatives to calculate the expected Jacobian
  {
    expected = numericalDerivative11<Rot3>(
        boost::bind(&RotateDirectionsFactor::evaluateError, &f, _1,
            boost::none), iRc);
    f.evaluateError(iRc, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
  {
    expected = numericalDerivative11<Rot3>(
        boost::bind(&RotateDirectionsFactor::evaluateError, &f, _1,
            boost::none), R);
    f.evaluateError(R, actual);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
}

//*************************************************************************
TEST (RotateDirectionsFactor, minimization) {
  // Let's try to recover the correct iRc by minimizing
  NonlinearFactorGraph graph;
  Model model = noiseModel::Isotropic::Sigma(2, 0.01);
  graph.add(RotateDirectionsFactor(1, p1, z1, model));
  graph.add(RotateDirectionsFactor(1, p2, z2, model));
  graph.add(RotateDirectionsFactor(1, p3, z3, model));

  // Check error at ground truth
  Values truth;
  truth.insert(1, iRc);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  double degree = M_PI / 180;
  Rot3 initialE = iRc.retract(degree * (Vector(3) << 20, -20, 20));
  initial.insert(1, initialE);

#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT_DOUBLES_EQUAL(3335.9, graph.error(initial), 1);
#else
  EXPECT_DOUBLES_EQUAL(3162, graph.error(initial), 1);
#endif

  // Optimize
  LevenbergMarquardtParams parameters;
  //parameters.setVerbosity("ERROR");
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  Rot3 actual = result.at<Rot3>(1);
  EXPECT(assert_equal(iRc, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

