/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ParallaxAnglePoint2.h
 * @brief  Parallax Angle Point without depth
 * @author Ellon P. Mendes
 */

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <gtsam/geometry/ParallaxAnglePointTools.h>

#include <boost/serialization/nvp.hpp>

#include <cmath>

namespace gtsam {

  /**
   * A Non-Depth Parallax Point
   * @addtogroup slam
   * \nosubgrouping
   */
  class GTSAM_EXPORT ParallaxAnglePoint2 : public DerivedValue<ParallaxAnglePoint2> {
  public:
    /// dimension of the variable - used to autodetect sizes
    static const size_t dimension = 2;

  private:
    double pitch_; ///< Angle around y-axis, in rad.
    double yaw_; ///< Angle around z-axis, in rad.

  public:

    /// @name Standard Constructors
    /// @{

    /// Default constructor creates a ParallaxAnglePoint2 oriented on x-axis.
    ParallaxAnglePoint2(): pitch_(0), yaw_(0) {}

    /// Construct from x, y, and z coordinates
    ParallaxAnglePoint2(double pitch, double yaw): pitch_(pitch), yaw_(yaw) {}

    /// @}
    /// @name Advanced Constructors
    /// @{

    /// Construct from 2-element vector
    ParallaxAnglePoint2(const Vector& v) {
      if(v.size() != 2)
        throw std::invalid_argument("ParallaxAnglePoint2 constructor from Vector requires that the Vector have dimension 2");
      pitch_ = v(0);
      yaw_ = v(1);
    }

    /// Construct from camera and measurement
    template<class PinholeCameraType>
    static ParallaxAnglePoint2 FromCameraAndMeasurement(const PinholeCameraType &camera, const Point2 &measurement)
    {
      Point3 vecFromMain = camera.backprojectPointAtInfinity(measurement);

      Vector2 py = vec2py(vecFromMain.vector());

      return ParallaxAnglePoint2(py(0),py(1));
    }

    /// Construct from pose, body to sensor pose and camera calibration
    template<class CalibrationType>
    static ParallaxAnglePoint2 FromPoseMeasurementAndCalibration(const Pose3 &pose, const Point2 &measurement, const CalibrationType& K, boost::optional<Pose3> body_P_sensor = boost::none)
    {
      boost::shared_ptr<PinholeCamera<CalibrationType> > camera_ptr;

      if(body_P_sensor)
      {
        camera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(pose.compose(*body_P_sensor), K);
      }
      else
      {
        camera_ptr = boost::make_shared<PinholeCamera<CalibrationType> >(pose, K);
      }

      Point3 vecFromMain = camera_ptr->backprojectPointAtInfinity(measurement);

      Vector2 py = vec2py(vecFromMain.vector());

      return ParallaxAnglePoint2(py(0),py(1));
    }

    /// @}
    /// @name Testable
    /// @{

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** equals with an tolerance */
    bool equals(const ParallaxAnglePoint2& p, double tol = 1e-9) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    inline static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    inline size_t dim() const { return dimension; }

    /// Updates a with tangent space delta
    inline ParallaxAnglePoint2 retract(const Vector& v) const { return ParallaxAnglePoint2(*this + v); }

    /// Returns inverse retraction
    inline Vector2 localCoordinates(const ParallaxAnglePoint2& q) const { return (q - *this).vector(); }

    /// @}
    /// @name Standard Interface
    /// @{

    /// assignment
    // ParallaxAnglePoint2 operator = (const ParallaxAnglePoint2& q) const;

    ///add two points
    ParallaxAnglePoint2 operator + (const ParallaxAnglePoint2& q) const;

    ///subtract two points
    ParallaxAnglePoint2 operator - (const ParallaxAnglePoint2& q) const;

    /** return vectorized form (column-wise)*/
    Vector2 vector() const { return Vector2(yaw_,pitch_); }

    /// get x
    inline double yaw() const {return yaw_;}

    /// get y
    inline double pitch() const {return pitch_;}

    /// @}

    Vector3 directionVector(boost::optional<gtsam::Matrix&> H = boost::none) const;

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const ParallaxAnglePoint2& p);

  private:

    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version)
    {
      ar & boost::serialization::make_nvp("ParallaxAnglePoint2",
          boost::serialization::base_object<Value>(*this));
      ar & BOOST_SERIALIZATION_NVP(yaw_);
      ar & BOOST_SERIALIZATION_NVP(pitch_);
    }

    /// @}

  };

}
