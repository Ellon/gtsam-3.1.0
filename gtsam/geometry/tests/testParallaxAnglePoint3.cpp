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
#include <gtsam/geometry/Cal3_S2.h>
#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;

const double tol = 1e-6;

#define radians(deg)  ((deg) * M_PI / 180.0)
#define degrees(rad)  ((rad) * 180.0 / M_PI)

GTSAM_CONCEPT_TESTABLE_INST(ParallaxAnglePoint3)
GTSAM_CONCEPT_MANIFOLD_INST(ParallaxAnglePoint3)

/* ************************************************************************* */
TEST(ParallaxAnglePoint3, constructor) {
  ParallaxAnglePoint3 p1(1, 2, 3);
  EXPECT_DOUBLES_EQUAL(p1.pitch(),1,tol);
  EXPECT_DOUBLES_EQUAL(p1.yaw(),2,tol);
  EXPECT_DOUBLES_EQUAL(p1.parallax(),3,tol);

  ParallaxAnglePoint3 p2 = p1;
  EXPECT(assert_equal(p1, p2));
  EXPECT_DOUBLES_EQUAL(p2.pitch(),1,tol);
  EXPECT_DOUBLES_EQUAL(p2.yaw(),2,tol);
  EXPECT_DOUBLES_EQUAL(p2.parallax(),3,tol);

  // Data to test static constructors
  Cal3_S2 K(320, 320, 0, 320, 240);
  Pose3 mainPose(Rot3(), Point3(2,0,0));
  Pose3 assoPose(Rot3::Ry(radians(-90.0)), Point3(10,0,0));
  Pose3 sensorPose(Rot3::RzRyRx(radians(-90.0),0,radians(-90.0)), Point3(0,0,3));
  Point2 measurementFromMain(320,240);
  Point2 measurementFromAsso(320,240);

  // test FromPosesMeasurementsAndCalibration
  ParallaxAnglePoint3 p3 = ParallaxAnglePoint3::FromPosesMeasurementsAndCalibration(
    mainPose, measurementFromMain,
    assoPose, measurementFromAsso,
    K, sensorPose);
  ParallaxAnglePoint3 ep3(0,0,radians(90.0));
  EXPECT(assert_equal(ep3,p3));

  // test FromCamerasAndMeasurements
  PinholeCamera<Cal3_S2> mainCamera(mainPose.compose(sensorPose), K);
  PinholeCamera<Cal3_S2> assoCamera(assoPose.compose(sensorPose), K);
  p3 = ParallaxAnglePoint3::FromCamerasAndMeasurements(
      mainCamera, measurementFromMain,
      assoCamera, measurementFromAsso);
  EXPECT(assert_equal(ep3,p3));

  // test FromParallaxAnglePointAndAnchors
  Point3 newMainAnchor(10,0,0);
  Point3 newAssoAnchor(10,0,3);
  ParallaxAnglePoint3 p4 = ParallaxAnglePoint3::FromParallaxAnglePointAndAnchors(ep3,
    mainCamera.pose().translation(), assoCamera.pose().translation(),
    newMainAnchor, newAssoAnchor);
  ParallaxAnglePoint3 ep4(radians(45.0),radians(180.0),radians(45.0));
  EXPECT(assert_equal(ep4,p4));


}

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
  // EXPECT(assert_equal( ParallaxAnglePoint3(-5,-6,-7), -ParallaxAnglePoint3(5,6,7) ));
  EXPECT(assert_equal( ParallaxAnglePoint3(5,6,7), ParallaxAnglePoint3(4,5,6)+ParallaxAnglePoint3(1,1,1)));
  EXPECT(assert_equal( ParallaxAnglePoint3(3,4,5), ParallaxAnglePoint3(4,5,6)-ParallaxAnglePoint3(1,1,1)));
  // EXPECT(assert_equal( ParallaxAnglePoint2(8,6), ParallaxAnglePoint2(4,3)*2));
  // EXPECT(assert_equal( ParallaxAnglePoint2(4,6), 2*ParallaxAnglePoint2(2,3)));
  // EXPECT(assert_equal( ParallaxAnglePoint2(2,3), ParallaxAnglePoint2(4,6)/2));
}

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, stream) {
  ParallaxAnglePoint3 p(1, 2, 3);
  std::ostringstream os;
  os << p;
  EXPECT(os.str() == "(1, 2, 3)");
}

namespace {
  // some shared test values. The 3D point is at the origin
  // check jacobiansPapVecFromAnchors.m for the computation of the jacobians below
  Point3 mainAnchor(0, -3, 0);
  Point3 assoAnchor(0, -5, 5);
  Point3 otheAnchor(7,  0, 0);
  ParallaxAnglePoint3 p(0, radians(90.0), radians(45.0));
  Vector expectedNormedVecFromMain = (-mainAnchor.vector()).normalized();
  Vector expectedNormedVecFromAsso = (-assoAnchor.vector()).normalized();
  Vector expectedNormedVecFromOthe = (-otheAnchor.vector()).normalized();

  Matrix eVECFROMMAIN_point = (Matrix(3,3) <<
                         0,    -1.000000000000000e+00,                         0,
                         0,     6.123233995736766e-17,                         0,
     1.000000000000000e+00,                         0,                         0);

  Matrix eVECFROMASSO_point = (Matrix(3,3) <<
     3.030846196824227e-16,    -2.121320343559643e+00,    -3.030846196824226e-16,
     4.949747468305834e+00,     8.659560562354935e-18,    -3.535533905932738e+00,
     2.121320343559643e+00,                         0,    -3.535533905932738e+00);

  Matrix eVECFROMASSO_main = (Matrix(3,3) <<
     7.071067811865475e-01,    -4.329780281177466e-17,    -4.329780281177467e-17,
    -6.061692393648452e-17,    -1.110223024625157e-16,    -7.071067811865477e-01,
                         0,                         0,     7.071067811865475e-01);

  Matrix eVECFROMASSO_asso = (Matrix(3,3) <<
    -7.071067811865475e-01,     4.329780281177466e-17,     4.329780281177467e-17,
     6.061692393648452e-17,     1.110223024625157e-16,     7.071067811865477e-01,
                         0,                         0,    -7.071067811865475e-01);


  Matrix eVECFROMOTHE_point = (Matrix(3,3) <<
    3.030846196824227e-16,    -2.121320343559643e+00,    -4.949747468305833e+00,
    4.949747468305834e+00,     8.659560562354935e-18,    -7.071067811865476e+00,
    2.121320343559643e+00,                         0,                         0);

  Matrix eVECFROMOTHE_main = (Matrix(3,3) <<
     7.071067811865475e-01,    -4.329780281177466e-17,    -4.329780281177467e-17,
    -6.061692393648452e-17,    -1.110223024625157e-16,    -7.071067811865477e-01,
                         0,                         0,     7.071067811865475e-01);

  Matrix eVECFROMOTHE_asso = (Matrix(3,3) <<
    3.711716093648717e-33,     4.329780281177466e-17,     4.329780281177467e-17,
    6.061692393648452e-17,     7.071067811865476e-01,     7.071067811865477e-01,
                        0,                         0,                         0);

  Matrix eVECFROMOTHE_othe = (Matrix(3,3) <<
    -7.071067811865475e-01,                         0,                         0,
                         0,    -7.071067811865475e-01,                         0,
                         0,                         0,    -7.071067811865475e-01);

}


/* ************************************************************************* */
TEST(ParallaxAnglePoint3, directionVectorFromMainAnchor) {

  // simple call
  EXPECT(assert_equal(expectedNormedVecFromMain, p.directionVectorFromMainAnchor()));

  // With jacobian
  Matrix VECFROMMAIN_point;
  EXPECT(assert_equal(expectedNormedVecFromMain, p.directionVectorFromMainAnchor(VECFROMMAIN_point)));
  EXPECT(assert_equal(eVECFROMMAIN_point, VECFROMMAIN_point ));

}

/* ************************************************************************* */
TEST(ParallaxAnglePoint3, directionVectorFromAssoAnchor) {

  // simple call
  EXPECT(assert_equal(expectedNormedVecFromAsso, p.directionVectorFromAssoAnchor(mainAnchor,assoAnchor).normalized()));

  // With jacobian
  Matrix VECFROMASSO_point, VECFROMASSO_main, VECFROMASSO_asso;
  EXPECT(assert_equal(expectedNormedVecFromAsso, p.directionVectorFromAssoAnchor(mainAnchor,assoAnchor, VECFROMASSO_point, VECFROMASSO_main, VECFROMASSO_asso).normalized()));

  EXPECT(assert_equal(eVECFROMASSO_point, VECFROMASSO_point ));
  EXPECT(assert_equal(eVECFROMASSO_main,  VECFROMASSO_main  ));
  EXPECT(assert_equal(eVECFROMASSO_asso,  VECFROMASSO_asso  ));

}

TEST(ParallaxAnglePoint3, directionVectorFromOtheAnchor) {

  // simple call
  EXPECT(assert_equal(expectedNormedVecFromOthe, p.directionVectorFromOtheAnchor(mainAnchor,assoAnchor,otheAnchor).normalized()));

  // With jacobian
  Matrix VECFROMOTHE_point, VECFROMOTHE_main, VECFROMOTHE_asso, VECFROMOTHE_othe;
  EXPECT(assert_equal(expectedNormedVecFromOthe, p.directionVectorFromOtheAnchor(mainAnchor,assoAnchor,otheAnchor, VECFROMOTHE_point, VECFROMOTHE_main, VECFROMOTHE_asso, VECFROMOTHE_othe).normalized()));

  EXPECT(assert_equal(eVECFROMOTHE_point, VECFROMOTHE_point ));
  EXPECT(assert_equal(eVECFROMOTHE_main,  VECFROMOTHE_main  ));
  EXPECT(assert_equal(eVECFROMOTHE_asso,  VECFROMOTHE_asso  ));
  EXPECT(assert_equal(eVECFROMOTHE_othe,  VECFROMOTHE_othe  ));

}

/* ************************************************************************* */
TEST( ParallaxAnglePoint3, toPoint3) {


  EXPECT(assert_equal(Point3(), p.toPoint3(mainAnchor, assoAnchor) ));

  Pose3 mainPose(Rot3(),mainAnchor - Point3(0,0,2));
  Pose3 assoPose(Rot3(),assoAnchor - Point3(0,0,2));
  Pose3 sensorPose(Rot3(),Point3(0,0,2));

  EXPECT(assert_equal(Point3(), p.toPoint3( Pose3(Rot3(),mainAnchor), Pose3(Rot3(),assoAnchor) )));
  EXPECT(assert_equal(Point3(), p.toPoint3( mainPose, assoPose, sensorPose )));

}

/* ************************************************************************* */
int main () {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

