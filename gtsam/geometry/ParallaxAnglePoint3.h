/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ParallaxAnglePoint3.h
 * @brief  3D Parallax Angle Point
 * @author Ellon P. Mendes
 */

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <gtsam/geometry/ParallaxAnglePointTools.h>

#include <boost/serialization/nvp.hpp>

#include <cmath>

namespace gtsam {

	  /**
   * A 3D Parallax Point
   * @addtogroup slam
   * \nosubgrouping
   */
  class GTSAM_EXPORT ParallaxAnglePoint3 : public DerivedValue<ParallaxAnglePoint3> {
  public:
    /// dimension of the variable - used to autodetect sizes
    static const size_t dimension = 3;

  private:
    double pitch_; ///< Angle around y-axis, in rad.
    double yaw_; ///< Angle around z-axis, in rad.
    double parallax_; ///< Angle between main anchor to feature vector and associated anchor to feature, in rad.

  public:

    /// @name Standard Constructors
    /// @{

    /// Default constructor creates a ParallaxAnglePoint3 oriented on x-axis, and with coincident anchors.
    ParallaxAnglePoint3(): pitch_(0), yaw_(0), parallax_(0) {}

    /// Construct from yaw, pitch, and parallax coordinates
    ParallaxAnglePoint3(double pitch, double yaw, double parallax): pitch_(pitch), yaw_(yaw), parallax_(parallax) {}

    /// @}
    /// @name Advanced Constructors
    /// @{

    /// Construct from 2-element vector
    ParallaxAnglePoint3(const Vector& v) {
      if(v.size() != 3)
        throw std::invalid_argument("ParallaxAnglePoint3 constructor from Vector requires that the Vector have dimension 3");
      pitch_ = v(0);
      yaw_ = v(1);
      parallax_ = v(2);
    }

    /// Construct from main and associated cameras and respective measurements
    template<class PinholeCameraType>
    static ParallaxAnglePoint3 FromCamerasAndMeasurements(
      const PinholeCameraType &mainCamera, const Point2 &measurementFromMain,
      const PinholeCameraType &assoCamera, const Point2 &measurementFromAsso)
    {
      // 'backprojectPointAtInfinity' returns the point already in the world frame
      Point3 pointFromMain = mainCamera.backprojectPointAtInfinity(measurementFromMain);
      Point3 pointFromAsso = assoCamera.backprojectPointAtInfinity(measurementFromAsso);

      Vector2 py = vec2py(pointFromMain.vector());

      double pitch = py(0);
      double yaw   = py(1);
      double parallax = vectors2angle(pointFromMain.vector(),pointFromAsso.vector());

      return ParallaxAnglePoint3(pitch,yaw,parallax);
    }

    /// Construct from main and associated poses, together with respective measurements, camera calibration and body to sensor pose
    template<class CalibrationType>
    static ParallaxAnglePoint3 FromPosesMeasurementsAndCalibration(
      const Pose3 &mainPose, const Point2 &measurementFromMain,
      const Pose3 &assoPose, const Point2 &measurementFromAsso,
      const CalibrationType& K, boost::optional<Pose3> body_P_sensor = boost::none)
    {
      boost::shared_ptr<PinholeCamera<CalibrationType> > mainCamera_ptr;
      boost::shared_ptr<PinholeCamera<CalibrationType> > assoCamera_ptr;

      if(body_P_sensor)
      {
        mainCamera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(mainPose.compose(*body_P_sensor), K);
        assoCamera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(assoPose.compose(*body_P_sensor), K);
      }
      else
      {
        mainCamera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(mainPose, K);
        assoCamera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(assoPose, K);
      }

      // 'backprojectPointAtInfinity' returns the point already in the world frame
      Point3 vecFromMain = mainCamera_ptr->backprojectPointAtInfinity(measurementFromMain);
      Point3 vecFromAsso = assoCamera_ptr->backprojectPointAtInfinity(measurementFromAsso);

      Vector2 py = vec2py(vecFromMain.vector());

      double pitch = py(0);
      double yaw   = py(1);
      double parallax = vectors2angle(vecFromMain.vector(),vecFromAsso.vector());

      return ParallaxAnglePoint3(pitch,yaw,parallax);
    }

    static ParallaxAnglePoint3 FromParallaxAnglePointAndAnchors(const ParallaxAnglePoint3 &oldPoint,
                        const Point3 &oldMainAnchor, const Point3 &oldAssoAnchor,
                        const Point3 &newMainAnchor, const Point3 &newAssoAnchor);

    /// @}
    /// @name Testable
    /// @{

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance */
    bool equals(const ParallaxAnglePoint3& p, double tol = 1e-9) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    inline static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    inline size_t dim() const { return dimension; }

    /// Updates a with tangent space delta
    ParallaxAnglePoint3 retract(const Vector& v) const;

    /// Returns inverse retraction
    inline Vector3 localCoordinates(const ParallaxAnglePoint3& q) const { return (q - *this).vector(); }

    /// @}
    /// @name Standard Interface
    /// @{

    /// assignment
    // ParallaxAnglePoint3 operator = (const ParallaxAnglePoint3& q) const;

    ///add two points
    ParallaxAnglePoint3 operator + (const ParallaxAnglePoint3& q) const;

    ///subtract two points
    ParallaxAnglePoint3 operator - (const ParallaxAnglePoint3& q) const;

    /** return vectorized form (column-wise)*/
    Vector3 vector() const { return Vector3(pitch_,yaw_,parallax_); }

    /// get yaw
    inline double yaw() const {return yaw_;}

    /// get pitch
    inline double pitch() const {return pitch_;}

    /// get parallax
    inline double parallax() const {return parallax_;}

    /// @}

    Vector3 directionVectorFromMainAnchor(boost::optional<gtsam::Matrix&> H = boost::none) const;

    Vector3 directionVectorFromAssoAnchor(
      const Point3 & mainAnchor, const Point3 & assoAnchor,
      boost::optional<gtsam::Matrix&> Dpoint = boost::none,
      boost::optional<gtsam::Matrix&> Dmain  = boost::none,
      boost::optional<gtsam::Matrix&> Dasso  = boost::none) const;

    Vector3 directionVectorFromOtheAnchor(
      const Point3 & mainAnchor, const Point3 & assoAnchor, const Point3 & otheAnchor,
      boost::optional<gtsam::Matrix&> Dpoint = boost::none,
      boost::optional<gtsam::Matrix&> Dmain  = boost::none,
      boost::optional<gtsam::Matrix&> Dasso  = boost::none,
      boost::optional<gtsam::Matrix&> Dothe  = boost::none) const;

    Point3 toPoint3(const Point3 & mainAnchor, const Point3 &  assoAnchor) const;

    Point3 toPoint3(const Pose3 & mainPose, const Pose3 & assoPose, boost::optional<const Pose3 &> body_P_sensor = boost::none) const;

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const ParallaxAnglePoint3& p);

  private:

    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp("ParallaxAnglePoint3",
          boost::serialization::base_object<Value>(*this));
      ar & BOOST_SERIALIZATION_NVP(pitch_);
      ar & BOOST_SERIALIZATION_NVP(yaw_);
      ar & BOOST_SERIALIZATION_NVP(parallax_);
    }

    /// @}

  };

}
