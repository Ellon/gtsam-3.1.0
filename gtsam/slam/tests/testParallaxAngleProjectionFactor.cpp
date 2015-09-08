/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testParallaxAngleProjectionFactor.cpp
 *  @brief Unit tests for ParallaxAngleProjectionFactor Classes
 *  @author Ellon Paiva Mendes
 *  @date Sep 2015
 */

#include <gtsam/slam/ProjectionFactor.h> // REMOVE THIS LINE
#include <gtsam/slam/ParallaxAngleProjectionFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// make a realistic calibration matrix
static double fov = 60; // degrees
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef ParallaxAngleMainAnchorProjectionFactor<Pose3, ParallaxAnglePoint3> TestProjectionToMainFactor;
typedef ParallaxAngleAssoAnchorProjectionFactor<Pose3, ParallaxAnglePoint3> TestProjectionToAssoFactor;
typedef ParallaxAngleProjectionFactor<Pose3, ParallaxAnglePoint3> TestProjectionFactor;


#define deg2rad(deg)  ((deg) * M_PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / M_PI)

// ############################################################################
// #####################  PROJECTION TO MAIN ANCHOR  ##########################
// ############################################################################

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, nonStandard ) {
  ParallaxAngleMainAnchorProjectionFactor<Pose3, ParallaxAnglePoint3, Cal3DS2> f;
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, Constructor) {
  Key poseKey(X(1));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);

  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K);
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, ConstructorWithTransform) {
  Key poseKey(X(1));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);

  TestProjectionToMainFactor factor1(measurement, model, X(1), L(1), K);
  TestProjectionToMainFactor factor2(measurement, model, X(1), L(1), K);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionToMainFactor factor1(measurement, model, X(1), L(1), K, body_P_sensor);
  TestProjectionToMainFactor factor2(measurement, model, X(1), L(1), K, body_P_sensor);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, Error ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key poseKey(X(1));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K);

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0,0,-6));
  // NOTE: parallax does not matter here. In fact, since pitch = M_PI_2, yaw also doesn't matter.
  ParallaxAnglePoint3 point(deg2rad(90), 0, 0);

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, ErrorWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key poseKey(X(1));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);

  // GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K);
  // Point3 point(0, 0, 0); // Parallax does not matter here

  // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
  Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
  ParallaxAnglePoint3 point(0, 0, 0); // Parallax does not matter here

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(pose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, Jacobian ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key poseKey(X(1));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K);

  // Set the linearization point
  Pose3 pose(Rot3(), Point3(0,0,-6));
  // NOTE: parallax does not matter here. In fact, since pitch = M_PI_2, yaw also doesn't matter.
  ParallaxAnglePoint3 point(deg2rad(90), 0, 0);

  // Use the factor to calculate the Jacobians
  Matrix ERROR_pose, ERROR_point;
  Vector actualError(factor.evaluateError(pose, point, ERROR_pose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // The expected Jacobians
  Matrix eERROR_pose = (Matrix(2, 6) <<
                         0,    -5.542560000000000e+02,                         0,                         0,                         0,                         0,
     5.542560000000000e+02,                         0,    -3.393839181541077e-14,                         0,                         0,                         0);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -5.542560000000000e+02,                         0,                         0,
                         0,     3.393839181541077e-14,                         0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_pose,  ERROR_pose,  1e-3));
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToMainFactor, JacobianWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key poseKey(X(1));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionToMainFactor factor(measurement, model, poseKey, pointKey, K, body_P_sensor);

  // Set the linearization point.
  Pose3 mainPose(Rot3::Rz(M_PI_2), Point3(-0.10, -6.25, -1.0));
  // Pose3 assoPose(Rot3::Rz(M_PI),   Point3( 6.25, -0.10, -1.0));
  // Pose3 othePose(Rot3(),           Point3(-6.25,  0.10, -1.0));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));


  // Use the factor to calculate the Jacobians
  Matrix ERROR_mainpose, ERROR_point;;
  Vector actualError(factor.evaluateError(mainPose, point, ERROR_mainpose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // The expected Jacobians
  Matrix eERROR_mainpose = (Matrix(2, 6) <<
     3.752090318385911e-46,    -7.041041480035329e-31,     5.542560000000000e+02,                         0,                         0,                         0,
     5.308492476261266e-14,    -5.542560000000000e+02,                         0,                         0,                         0,                         0);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -7.535836802443800e-30,    -5.542560000000000e+02,                         0,
    -5.542560000000000e+02,                         0,                         0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_mainpose,  ERROR_mainpose,  1e-3));
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));
}

// ############################################################################
// ##################  PROJECTION TO ASSOCIATE ANCHOR  ########################
// ############################################################################

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, nonStandard ) {
  ParallaxAngleAssoAnchorProjectionFactor<Pose3, ParallaxAnglePoint3, Cal3DS2> f;
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, Constructor) {
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);

  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K);
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, ConstructorWithTransform) {
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K, body_P_sensor);
}

TEST( ParallaxAngleProjectionToAssoFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);

  TestProjectionToAssoFactor factor1(measurement, model, X(1), X(2), L(1), K);
  TestProjectionToAssoFactor factor2(measurement, model, X(1), X(2), L(1), K);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionToAssoFactor factor1(measurement, model, X(1), X(2), L(1), K, body_P_sensor);
  TestProjectionToAssoFactor factor2(measurement, model, X(1), X(2), L(1), K, body_P_sensor);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, Error ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(0,-6,0));
  Pose3 assoPose(Rot3(), Point3(0,0,-6));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(mainPose, assoPose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, ErrorWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K, body_P_sensor);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(-0.25, -5.90, -1.0));
  Pose3 assoPose(Rot3(), Point3(-6.25, 0.10 , -1.0));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(mainPose, assoPose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));

}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, Jacobian ) {

  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(0,-6,0));
  Pose3 assoPose(Rot3(), Point3(0,0,-6));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Matrix ERROR_mainpose, ERROR_assopose, ERROR_point;
  Vector actualError(factor.evaluateError(mainPose, assoPose, point, ERROR_mainpose, ERROR_assopose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // The expected Jacobians
  Matrix eERROR_mainpose = (Matrix(2, 6) <<
    0,                         0,                         0,     9.237599999999999e+01,    -5.656398635901795e-15,    -5.656398635901795e-15,
    0,                         0,                         0,    -5.656398635901795e-15,    -2.051159242455469e-14,     1.538369431841602e-14);
  Matrix eERROR_assopose = (Matrix(2, 6) <<
                         0,    -5.542560000000000e+02,                         0,    -9.237599999999999e+01,     5.656398635901795e-15,     5.656398635901795e-15,
     5.542560000000000e+02,                         0,    -3.393839181541077e-14,     5.656398635901795e-15,     2.051159242455469e-14,    -1.538369431841602e-14);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -6.787678363082154e-14,    -5.542560000000000e+02,    -3.393839181541076e-14,
    -5.542560000000000e+02,     6.787678363082154e-14,    -5.542559999999999e+02);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_mainpose,  ERROR_mainpose,  1e-3));
  EXPECT(assert_equal(eERROR_assopose,  ERROR_assopose,  1e-3));
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));

}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionToAssoFactor, JacobianWithTransform ) {

  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionToAssoFactor factor(measurement, model, mainKey, assoKey, pointKey, K, body_P_sensor);

  // Set the linearization point
  Pose3 mainPose(Rot3::Rz(M_PI_2), Point3(-0.10, -6.25, -1.0));
  Pose3 assoPose(Rot3::Rz(M_PI),   Point3( 6.25, -0.10, -1.0));
  // Pose3 othePose(Rot3(),           Point3(-6.25,  0.10, -1.0));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));


  // Use the factor to calculate the error
  Matrix ERROR_mainpose, ERROR_assopose, ERROR_point;
  Vector actualError(factor.evaluateError(mainPose, assoPose, point, ERROR_mainpose, ERROR_assopose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // The expected Jacobians
  Matrix eERROR_mainpose = (Matrix(2, 6) <<
    -1.487090450780215e-14,    -1.410171979188135e-14,     2.563949053069337e-15,    -2.563949053069336e-14,    -2.051159242455469e-14,    -5.127898106138673e-15,
     9.237600000000000e+00,     2.309400000000000e+01,    -7.888609052210118e-31,     6.310887241768094e-30,     2.277244218146755e-30,    -9.237599999999999e+01);
  Matrix eERROR_assopose = (Matrix(2, 6) <<
     2.512670072007950e-14,    -1.153777073881202e-14,     5.542560000000000e+02,     2.563949053069336e-14,     2.051159242455469e-14,     5.127898106138673e-15,
    -9.237599999999938e+00,    -5.773500000000000e+02,    -6.153477727366407e-14,    -6.310887241768094e-30,    -2.277244218146755e-30,     9.237599999999999e+01);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -3.076738863683204e-14,    -5.542560000000000e+02,    -5.542559999999999e+02,
    -5.542560000000000e+02,     6.153477727366409e-14,     6.153477727366406e-14);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_mainpose,  ERROR_mainpose,  1e-3)); // NOT_EQUAL
  EXPECT(assert_equal(eERROR_assopose,  ERROR_assopose,  1e-3)); // NOT_EQUAL
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));

}

// ############################################################################
// ####################  PROJECTION TO OTHER ANCHOR  ##########################
// ############################################################################

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, nonStandard ) {
  ParallaxAngleProjectionFactor<Pose3, ParallaxAnglePoint3, Cal3DS2> f;
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, Constructor) {
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);

  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K);
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, ConstructorWithTransform) {
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));

  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K, body_P_sensor);
}

TEST( ParallaxAngleProjectionFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);

  TestProjectionFactor factor1(measurement, model, X(1), X(2), X(3), L(1), K);
  TestProjectionFactor factor2(measurement, model, X(1), X(2), X(3), L(1), K);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, EqualsWithTransform ) {
  // Create two identical factors and make sure they're equal
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

  TestProjectionFactor factor1(measurement, model, X(1), X(2), X(3), L(1), K, body_P_sensor);
  TestProjectionFactor factor2(measurement, model, X(1), X(2), X(3), L(1), K, body_P_sensor);

  EXPECT(assert_equal(factor1, factor2));
}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, Error ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(0,-6,0));
  Pose3 assoPose(Rot3(), Point3(-6,0,0));
  Pose3 othePose(Rot3(), Point3(0,0,-6));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(mainPose, assoPose, othePose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));
}

// /* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, ErrorWithTransform ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K, body_P_sensor);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(-0.25, -5.90, -1.0));
  Pose3 assoPose(Rot3(), Point3( 5.75,  0.10, -1.0));
  Pose3 othePose(Rot3(), Point3(-6.25,  0.10, -1.0));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError(mainPose, assoPose, othePose, point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedError, actualError, 1e-9));

}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, Jacobian ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K);

  // Set the linearization point
  Pose3 mainPose(Rot3(), Point3(0,-6,0));
  Pose3 assoPose(Rot3(), Point3(-6,0,0));
  Pose3 othePose(Rot3(), Point3(0,0,-6));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Matrix ERROR_mainpose, ERROR_assopose, ERROR_othepose, ERROR_point;
  Vector actualError(factor.evaluateError(mainPose, assoPose, othePose, point, ERROR_mainpose, ERROR_assopose, ERROR_othepose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // The expected Jacobians
  Matrix eERROR_mainpose = (Matrix(2, 6) <<
    0,                         0,                         0,     9.237599999999999e+01,    -5.656398635901795e-15,    -5.656398635901795e-15,
    0,                         0,                         0,     1.025579621227735e-14,    -2.051159242455469e-14,                         0);
  Matrix eERROR_assopose = (Matrix(2, 6) <<
    0,                         0,                         0,    -5.693110545366889e-31,     5.656398635901795e-15,                         0,
    0,                         0,                         0,    -1.025579621227735e-14,     9.237600000000002e+01,                         0);
  Matrix eERROR_othepose = (Matrix(2, 6) <<
                        0,    -5.542560000000000e+02,                         0,    -9.237599999999999e+01,                         0,     5.656398635901795e-15,
    5.542560000000000e+02,                         0,    -3.393839181541077e-14,                         0,    -9.237599999999999e+01,                         0);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -3.393839181541077e-14,    -5.542560000000000e+02,    -3.393839181541076e-14,
                         0,     5.542560000000000e+02,    -5.542559999999999e+02);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_mainpose,  ERROR_mainpose,  1e-3));
  EXPECT(assert_equal(eERROR_assopose,  ERROR_assopose,  1e-3));
  EXPECT(assert_equal(eERROR_othepose,  ERROR_othepose,  1e-3));
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));

}

/* ************************************************************************* */
TEST( ParallaxAngleProjectionFactor, JacobianWithTransform ) {

  // Create the factor with a measurement that is 3 pixels off in x
  Key mainKey(X(1));
  Key assoKey(X(2));
  Key otheKey(X(3));
  Key pointKey(L(1));
  Point2 measurement(323.0, 240.0);
  Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
  TestProjectionFactor factor(measurement, model, mainKey, assoKey, otheKey, pointKey, K, body_P_sensor);

  // Set the linearization point
  Pose3 mainPose(Rot3::Rz(M_PI_2), Point3(-0.10, -6.25, -1.0));
  Pose3 assoPose(Rot3::Rz(M_PI),   Point3( 6.25, -0.10, -1.0));
  Pose3 othePose(Rot3(),           Point3(-6.25,  0.10, -1.0));
  ParallaxAnglePoint3 point(deg2rad(0), deg2rad(90), deg2rad(90));

  // Use the factor to calculate the error
  Matrix ERROR_mainpose, ERROR_assopose, ERROR_othepose, ERROR_point;
  Vector actualError(factor.evaluateError(mainPose, assoPose, othePose, point, ERROR_mainpose, ERROR_assopose, ERROR_othepose, ERROR_point));

  // The expected error is (-3.0, 0.0) pixels / UnitCovariance
  Vector expectedError = (Vector(2) << -3.0, 0.0);

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9)); // NOT_EQUAL

  // The expected Jacobians
  Matrix eERROR_mainpose = (Matrix(2, 6) <<
     1.538369431841602e-14,     1.538369431841602e-14,    -2.563949053069337e-15,     2.563949053069336e-14,     2.051159242455469e-14,    -4.554488436293512e-30,
     9.237599999999999e+00,     2.309400000000000e+01,     7.888609052210118e-31,    -3.155443620884047e-30,     1.226136156284215e-45,    -9.237599999999999e+01);
  Matrix eERROR_assopose = (Matrix(2, 6) <<
    -9.237599999999999e+01,                         0,     2.309400000000000e+01,    -2.563949053069336e-14,    -9.237600000000002e+01,                         0,
    -4.554488436293512e-30,                         0,     1.138622109073378e-30,    -1.264124481840521e-45,    -4.554488436293512e-30,                         0);
  Matrix eERROR_othepose = (Matrix(2, 6) <<
    -9.237599999999999e+01,    -2.524354896707238e-29,     5.773500000000000e+02,     3.155443620884047e-30,     9.237599999999999e+01,     4.554488436293512e-30,
    -9.237600000000000e+00,    -5.773500000000000e+02,     2.839899258795642e-29,     3.155443620884047e-30,     4.554488436293512e-30,     9.237599999999999e+01);
  Matrix eERROR_point = (Matrix(2, 3) <<
    -2.732693061776107e-29,     5.542559999999999e+02,     5.542559999999999e+02,
    -5.542560000000000e+02,     5.048709793414476e-29,     2.732693061776106e-29);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(eERROR_mainpose,  ERROR_mainpose,  1e-3)); // NOT_EQUAL
  EXPECT(assert_equal(eERROR_assopose,  ERROR_assopose,  1e-3)); // NOT_EQUAL
  EXPECT(assert_equal(eERROR_othepose,  ERROR_othepose,  1e-3)); // NOT_EQUAL
  EXPECT(assert_equal(eERROR_point, ERROR_point, 1e-3));

}




/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

