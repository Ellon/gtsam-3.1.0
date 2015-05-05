/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testGeneralSFMFactor.cpp
 * @date Dec 27, 2010
 * @author nikai
 * @brief unit tests for GeneralSFMFactor
 */

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/base/Testable.h>

#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>
using namespace boost;

#include <iostream>
#include <vector>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PinholeCamera<Cal3_S2> GeneralCamera;
typedef GeneralSFMFactor<GeneralCamera, Point3> Projection;
typedef NonlinearEquality<GeneralCamera> CameraConstraint;
typedef NonlinearEquality<Point3> Point3Constraint;

class Graph: public NonlinearFactorGraph {
public:
  void addMeasurement(int i, int j, const Point2& z, const SharedNoiseModel& model) {
    push_back(boost::make_shared<Projection>(z, model, X(i), L(j)));
  }

  void addCameraConstraint(int j, const GeneralCamera& p) {
    boost::shared_ptr<CameraConstraint> factor(new CameraConstraint(X(j), p));
    push_back(factor);
  }

  void addPoint3Constraint(int j, const Point3& p) {
    boost::shared_ptr<Point3Constraint> factor(new Point3Constraint(L(j), p));
    push_back(factor);
  }

};

static double getGaussian()
{
    double S,V1,V2;
    // Use Box-Muller method to create gauss noise from uniform noise
    do
    {
        double U1 = rand() / (double)(RAND_MAX);
        double U2 = rand() / (double)(RAND_MAX);
        V1 = 2 * U1 - 1;           /* V1=[-1,1] */
        V2 = 2 * U2 - 1;           /* V2=[-1,1] */
        S  = V1 * V1 + V2 * V2;
    } while(S>=1);
    return sqrt(-2.0f * (double)log(S) / S) * V1;
}

static const SharedNoiseModel sigma1(noiseModel::Unit::Create(2));

/* ************************************************************************* */
TEST( GeneralSFMFactor, equals )
{
  // Create two identical factors and make sure they're equal
  Vector z = (Vector(2) << 323.,240.);
  const Symbol cameraFrameNumber('x',1), landmarkNumber('l',1);
  const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
  boost::shared_ptr<Projection>
    factor1(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  boost::shared_ptr<Projection>
    factor2(new Projection(z, sigma, cameraFrameNumber, landmarkNumber));

  EXPECT(assert_equal(*factor1, *factor2));
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, error ) {
  Point2 z(3.,0.);
  const SharedNoiseModel sigma(noiseModel::Unit::Create(1));
  boost::shared_ptr<Projection>  factor(new Projection(z, sigma, X(1), L(1)));
  // For the following configuration, the factor predicts 320,240
  Values values;
  Rot3 R;
  Point3 t1(0,0,-6);
  Pose3 x1(R,t1);
  values.insert(X(1), GeneralCamera(x1));
  Point3 l1;  values.insert(L(1), l1);
  EXPECT(assert_equal(((Vector) (Vector(2) << -3.0, 0.0)), factor->unwhitenedError(values)));
}

static const double baseline = 5.0 ;

/* ************************************************************************* */
static vector<Point3> genPoint3() {
  const double z = 5;
  vector<Point3> landmarks ;
  landmarks.push_back(Point3 (-1.0,-1.0, z));
  landmarks.push_back(Point3 (-1.0, 1.0, z));
  landmarks.push_back(Point3 ( 1.0, 1.0, z));
  landmarks.push_back(Point3 ( 1.0,-1.0, z));
  landmarks.push_back(Point3 (-1.5,-1.5, 1.5*z));
  landmarks.push_back(Point3 (-1.5, 1.5, 1.5*z));
  landmarks.push_back(Point3 ( 1.5, 1.5, 1.5*z));
  landmarks.push_back(Point3 ( 1.5,-1.5, 1.5*z));
  landmarks.push_back(Point3 (-2.0,-2.0, 2*z));
  landmarks.push_back(Point3 (-2.0, 2.0, 2*z));
  landmarks.push_back(Point3 ( 2.0, 2.0, 2*z));
  landmarks.push_back(Point3 ( 2.0,-2.0, 2*z));
  return landmarks ;
}

static vector<GeneralCamera> genCameraDefaultCalibration() {
  vector<GeneralCamera> X ;
  X.push_back(GeneralCamera(Pose3(eye(3),Point3(-baseline/2.0, 0.0, 0.0))));
  X.push_back(GeneralCamera(Pose3(eye(3),Point3( baseline/2.0, 0.0, 0.0))));
  return X ;
}

static vector<GeneralCamera> genCameraVariableCalibration() {
  const Cal3_S2 K(640,480,0.01,320,240);
  vector<GeneralCamera> X ;
  X.push_back(GeneralCamera(Pose3(eye(3),Point3(-baseline/2.0, 0.0, 0.0)), K));
  X.push_back(GeneralCamera(Pose3(eye(3),Point3( baseline/2.0, 0.0, 0.0)), K));
  return X ;
}

static boost::shared_ptr<Ordering> getOrdering(const vector<GeneralCamera>& cameras, const vector<Point3>& landmarks) {
  boost::shared_ptr<Ordering> ordering(new Ordering);
  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) ordering->push_back(L(i)) ;
  for ( size_t i = 0 ; i < cameras.size() ; ++i ) ordering->push_back(X(i)) ;
  return ordering ;
}


/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_defaultK ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraDefaultCalibration();

  // add measurement with noise
  Graph graph;
  for ( size_t j = 0 ; j < cameras.size() ; ++j) {
    for ( size_t i = 0 ; i < landmarks.size() ; ++i) {
      Point2 pt = cameras[j].project(landmarks[i]) ;
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size()*landmarks.size() ;

  // add initial
  const double noise = baseline*0.1;
  Values values;
  for ( size_t i = 0 ; i < cameras.size() ; ++i )
    values.insert(X(i), cameras[i]) ;

  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) {
    Point3 pt(landmarks[i].x()+noise*getGaussian(),
              landmarks[i].y()+noise*getGaussian(),
              landmarks[i].z()+noise*getGaussian());
    values.insert(L(i), pt) ;
  }

  graph.addCameraConstraint(0, cameras[0]);

  // Create an ordering of the variables
  Ordering ordering = *getOrdering(cameras,landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * 1e-5 * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_SingleMeasurementError ) {
  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();
  // add measurement with noise
  Graph graph;
  for ( size_t j = 0 ; j < cameras.size() ; ++j) {
    for ( size_t i = 0 ; i < landmarks.size() ; ++i) {
      Point2 pt = cameras[j].project(landmarks[i]) ;
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size()*landmarks.size() ;

  // add initial
  const double noise = baseline*0.1;
  Values values;
  for ( size_t i = 0 ; i < cameras.size() ; ++i )
    values.insert(X(i), cameras[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) {
    if ( i == 0 ) {
      Point3 pt(landmarks[i].x()+noise*getGaussian(),
                landmarks[i].y()+noise*getGaussian(),
                landmarks[i].z()+noise*getGaussian());
      values.insert(L(i), pt) ;
    }
    else {
      values.insert(L(i), landmarks[i]) ;
    }
  }

  graph.addCameraConstraint(0, cameras[0]);
  const double reproj_error = 1e-5;

  Ordering ordering = *getOrdering(cameras,landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixCameras ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  const double noise = baseline*0.1;
  Graph graph;
  for ( size_t j = 0 ; j < cameras.size() ; ++j) {
    for ( size_t i = 0 ; i < landmarks.size() ; ++i) {
      Point2 pt = cameras[j].project(landmarks[i]) ;
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = landmarks.size()*cameras.size();

  Values values;
  for ( size_t i = 0 ; i < cameras.size() ; ++i )
    values.insert(X(i), cameras[i]) ;

  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) {
    Point3 pt(landmarks[i].x()+noise*getGaussian(),
              landmarks[i].y()+noise*getGaussian(),
              landmarks[i].z()+noise*getGaussian());
    //Point3 pt(landmarks[i].x(), landmarks[i].y(), landmarks[i].z());
    values.insert(L(i), pt) ;
  }

  for ( size_t i = 0 ; i < cameras.size() ; ++i )
    graph.addCameraConstraint(i, cameras[i]);

  const double reproj_error = 1e-5 ;

  Ordering ordering = *getOrdering(cameras,landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_FixLandmarks ) {

  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  Graph graph;
  for ( size_t j = 0 ; j < cameras.size() ; ++j) {
    for ( size_t i = 0 ; i < landmarks.size() ; ++i) {
      Point2 pt = cameras[j].project(landmarks[i]) ;
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = landmarks.size()*cameras.size();

  Values values;
  for ( size_t i = 0 ; i < cameras.size() ; ++i ) {
    const double
      rot_noise = 1e-5,
      trans_noise = 1e-3,
      focal_noise = 1,
      skew_noise = 1e-5;
    if ( i == 0 ) {
      values.insert(X(i), cameras[i]) ;
    }
    else {

      Vector delta = (Vector(11) <<
          rot_noise, rot_noise, rot_noise, // rotation
          trans_noise, trans_noise, trans_noise, // translation
          focal_noise, focal_noise, // f_x, f_y
          skew_noise, // s
          trans_noise, trans_noise // ux, uy
          ) ;
      values.insert(X(i), cameras[i].retract(delta)) ;
    }
  }

  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) {
    values.insert(L(i), landmarks[i]) ;
  }

  // fix X0 and all landmarks, allow only the cameras[1] to move
  graph.addCameraConstraint(0, cameras[0]);
  for ( size_t i = 0 ; i < landmarks.size() ; ++i )
    graph.addPoint3Constraint(i, landmarks[i]);

  const double reproj_error = 1e-5 ;

  Ordering ordering = *getOrdering(cameras,landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST( GeneralSFMFactor, optimize_varK_BA ) {
  vector<Point3> landmarks = genPoint3();
  vector<GeneralCamera> cameras = genCameraVariableCalibration();

  // add measurement with noise
  Graph graph;
  for ( size_t j = 0 ; j < cameras.size() ; ++j) {
    for ( size_t i = 0 ; i < landmarks.size() ; ++i) {
      Point2 pt = cameras[j].project(landmarks[i]) ;
      graph.addMeasurement(j, i, pt, sigma1);
    }
  }

  const size_t nMeasurements = cameras.size()*landmarks.size() ;

  // add initial
  const double noise = baseline*0.1;
  Values values;
  for ( size_t i = 0 ; i < cameras.size() ; ++i )
    values.insert(X(i), cameras[i]) ;

  // add noise only to the first landmark
  for ( size_t i = 0 ; i < landmarks.size() ; ++i ) {
    Point3 pt(landmarks[i].x()+noise*getGaussian(),
              landmarks[i].y()+noise*getGaussian(),
              landmarks[i].z()+noise*getGaussian());
    values.insert(L(i), pt) ;
  }

  // Constrain position of system with the first camera constrained to the origin
  graph.addCameraConstraint(0, cameras[0]);

  // Constrain the scale of the problem with a soft range factor of 1m between the cameras
  graph.push_back(RangeFactor<GeneralCamera,GeneralCamera>(X(0), X(1), 2.0, noiseModel::Isotropic::Sigma(1, 10.0)));

  const double reproj_error = 1e-5 ;

  Ordering ordering = *getOrdering(cameras,landmarks);
  LevenbergMarquardtOptimizer optimizer(graph, values, ordering);
  Values final = optimizer.optimize();
  EXPECT(optimizer.error() < 0.5 * reproj_error * nMeasurements);
}

/* ************************************************************************* */
TEST(GeneralSFMFactor, GeneralCameraPoseRange) {
  // Tests range factor between a GeneralCamera and a Pose3
  Graph graph;
  graph.addCameraConstraint(0, GeneralCamera());
  graph.push_back(RangeFactor<GeneralCamera, Pose3>(X(0), X(1), 2.0, noiseModel::Isotropic::Sigma(1, 1.0)));
  graph.push_back(PriorFactor<Pose3>(X(1), Pose3(Rot3(), Point3(1.0, 0.0, 0.0)), noiseModel::Isotropic::Sigma(6, 1.0)));

  Values init;
  init.insert(X(0), GeneralCamera());
  init.insert(X(1), Pose3(Rot3(), Point3(1.0,1.0,1.0)));

  // The optimal value between the 2m range factor and 1m prior is 1.5m
  Values expected;
  expected.insert(X(0), GeneralCamera());
  expected.insert(X(1), Pose3(Rot3(), Point3(1.5,0.0,0.0)));

  LevenbergMarquardtParams params;
  params.absoluteErrorTol = 1e-9;
  params.relativeErrorTol = 1e-9;
  Values actual = LevenbergMarquardtOptimizer(graph, init, params).optimize();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
TEST(GeneralSFMFactor, CalibratedCameraPoseRange) {
  // Tests range factor between a CalibratedCamera and a Pose3
  NonlinearFactorGraph graph;
  graph.push_back(PriorFactor<CalibratedCamera>(X(0), CalibratedCamera(), noiseModel::Isotropic::Sigma(6, 1.0)));
  graph.push_back(RangeFactor<CalibratedCamera, Pose3>(X(0), X(1), 2.0, noiseModel::Isotropic::Sigma(1, 1.0)));
  graph.push_back(PriorFactor<Pose3>(X(1), Pose3(Rot3(), Point3(1.0, 0.0, 0.0)), noiseModel::Isotropic::Sigma(6, 1.0)));

  Values init;
  init.insert(X(0), CalibratedCamera());
  init.insert(X(1), Pose3(Rot3(), Point3(1.0,1.0,1.0)));

  Values expected;
  expected.insert(X(0), CalibratedCamera(Pose3(Rot3(), Point3(-0.333333333333, 0, 0))));
  expected.insert(X(1), Pose3(Rot3(), Point3(1.333333333333, 0, 0)));

  LevenbergMarquardtParams params;
  params.absoluteErrorTol = 1e-9;
  params.relativeErrorTol = 1e-9;
  Values actual = LevenbergMarquardtOptimizer(graph, init, params).optimize();

  EXPECT(assert_equal(expected, actual, 1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
