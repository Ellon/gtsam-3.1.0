/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * GTSAM Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * Parallax Angle Projection Factor by Ellon P. Mendes

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ParallaxAngleProjectionFactor.h
 * @brief Parallax Angle projection factors
 * @author Ellon P. Mendes
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/optional.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/ParallaxAnglePoint2.h>
#include <gtsam/geometry/ParallaxAnglePoint3.h>

namespace gtsam {

  /** Non-linear factor for a 2D measurement to a Parallax Angle Point with only one anchor.
   *
   * Non-linear factor for a constraint derived from a 2D measurement bewteen a robot pose and a
   * landmark parametrized as Parallax Angle Point with no depth (a.k.a ParallaxAnglePoint2). The
   * measurement was taken from the anchor, and the calibration is known here.
   *
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class ParallaxAngleSingleAnchorProjectionFactor: public NoiseModelFactor2<POSE, LANDMARK>
  {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor2<POSE, LANDMARK> Base;

    /// shorthand for this class
    typedef ParallaxAngleSingleAnchorProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    ParallaxAngleSingleAnchorProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of main anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    ParallaxAngleSingleAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, pointKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of main anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    ParallaxAngleSingleAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, pointKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ParallaxAngleSingleAnchorProjectionFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ParallaxAngleSingleAnchorProjectionFactor, z = ";
      measured_.print();
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && this->measured_.equals(e->measured_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& mainPose, const ParallaxAnglePoint2& point,
        boost::optional<Matrix&> Dmain  = boost::none,
        boost::optional<Matrix&> Dpoint = boost::none) const {
      try {
        // Test if we need jacobians
        if (!Dmain && !Dpoint)
        {
          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Pose3 mainAnchorPose ( mainPose.compose(*body_P_sensor_) );

            // Get the direction to the point from observation point
            Point3 obs_T_point( point.directionVector() );

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
          else
          {
            // Get the direction to the point from observation point
            Point3 obs_T_point( point.directionVector() );

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {
            // Get the main and associated anchors, and the camera pose
            gtsam::Matrix MAINANCHORPOSE_mainpose;
            Pose3 mainAnchorPose ( mainPose.compose(*body_P_sensor_, MAINANCHORPOSE_mainpose) );

            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point;
            Point3 obs_T_point(point.directionVector( OBS_T_POINT_point ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_mainanchorori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_mainanchorori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Matrix PROJ_mainanchorpose(2,6);
            PROJ_mainanchorpose << PROJ_mainanchorori.block(0,0,2,3), zeros(2,3);
            Dmain->resize(2,6);
            *Dmain << (PROJ_mainanchorpose * MAINANCHORPOSE_mainpose);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,2);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

        }
        else
        {
            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point;
            Point3 obs_T_point(point.directionVector( OBS_T_POINT_point ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_mainori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_mainori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PROJ_mainori.block(0,0,2,3), zeros(2,3);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,2);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();
        }
      } catch( CheiralityException& e)
      {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,2);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
              " with single anchor (" << DefaultKeyFormatter(this->key1()) << ")" <<
              " moved behind camera " << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return ones(2) * 2.0 * K_->fx();
    }

        /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& mainPose, const ParallaxAnglePoint3& point,
        boost::optional<Matrix&> Dmain  = boost::none,
        boost::optional<Matrix&> Dpoint = boost::none) const {
      try {
        // Test if we need jacobians
        if (!Dmain && !Dpoint)
        {
          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Pose3 mainAnchorPose ( mainPose.compose(*body_P_sensor_) );

            // Get the direction to the point from observation point
            Point3 obs_T_point( point.directionVectorFromMainAnchor() );

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
          else
          {
            // Get the direction to the point from observation point
            Point3 obs_T_point( point.directionVectorFromMainAnchor() );

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {
            // Get the main and associated anchors, and the camera pose
            gtsam::Matrix MAINANCHORPOSE_mainpose;
            Pose3 mainAnchorPose ( mainPose.compose(*body_P_sensor_, MAINANCHORPOSE_mainpose) );

            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point;
            Point3 obs_T_point(point.directionVectorFromMainAnchor( OBS_T_POINT_point ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_mainanchorori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_mainanchorori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Matrix PROJ_mainanchorpose(2,6);
            PROJ_mainanchorpose << PROJ_mainanchorori.block(0,0,2,3), zeros(2,3);
            Dmain->resize(2,6);
            *Dmain << (PROJ_mainanchorpose * MAINANCHORPOSE_mainpose);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

        }
        else
        {
            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point;
            Point3 obs_T_point(point.directionVectorFromMainAnchor( OBS_T_POINT_point ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(mainPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_mainori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_mainori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PROJ_mainori.block(0,0,2,3), zeros(2,3);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();
        }
      } catch( CheiralityException& e)
      {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
              " with single anchor (" << DefaultKeyFormatter(this->key1()) << ")" <<
              " moved behind camera " << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return ones(2) * 2.0 * K_->fx();
    }


    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };
















  /** Non-linear factor for a 2D measurement to a Parallax Angle Point with only the anchors.
   *
   * Non-linear factor for a constraint derived from a 2D measurement bewteen a robot pose and a
   * landmark parametrized as 3D Parallax Angle Point (a.k.a ParallaxAnglePoint3). The measurement
   * was taken from the associated anchor, and the calibration is known here.
   *
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class ParallaxAngleOnlyAnchorsProjectionFactor: public NoiseModelFactor3<POSE, POSE, LANDMARK> {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor3<POSE, POSE, LANDMARK> Base;

    /// shorthand for this class
    typedef ParallaxAngleOnlyAnchorsProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    ParallaxAngleOnlyAnchorsProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of the main anchor
     * @param associatedAnchorKey is the index of the associated anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    ParallaxAngleOnlyAnchorsProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key associatedAnchorKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, associatedAnchorKey, pointKey),
          measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of the main anchor
     * @param associatedAnchorKey is the index of the associated anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    ParallaxAngleOnlyAnchorsProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key associatedAnchorKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, associatedAnchorKey, pointKey),
          measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ParallaxAngleOnlyAnchorsProjectionFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ParallaxAngleOnlyAnchorsProjectionFactor, z = ";
      measured_.print();
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && this->measured_.equals(e->measured_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& mainPose, const Pose3& assoPose, const ParallaxAnglePoint3& point,
        boost::optional<Matrix&> Dmain  = boost::none,
        boost::optional<Matrix&> Dasso  = boost::none,
        boost::optional<Matrix&> Dpoint = boost::none) const {
      try {
        // Test if we need jacobians
        if (!Dmain && !Dasso && !Dpoint)
        {
          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Point3 mainAnchor     ( mainPose.compose(*body_P_sensor_).translation() );
            Pose3  assoAnchorPose ( assoPose.compose(*body_P_sensor_)               );

            // Get the direction to the point from observation point
            Point3 obs_T_point(point.directionVectorFromAssoAnchor( mainAnchor, assoAnchorPose.translation() ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(assoAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
          else
          {
            // Get the direction to the point from observation point
            Point3 obs_T_point(point.directionVectorFromAssoAnchor( mainPose.translation(), assoPose.translation() ));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(assoPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {
            // Get the main and associated anchors, and the camera pose
            gtsam::Matrix MAINANCHORPOSE_mainpose, ASSOANCHORPOSE_assopose;
            Point3 mainAnchor     ( mainPose.compose(*body_P_sensor_, MAINANCHORPOSE_mainpose).translation() );
            Pose3  assoAnchorPose ( assoPose.compose(*body_P_sensor_, ASSOANCHORPOSE_assopose)               );

            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point, OBS_T_POINT_mainanchorpos, OBS_T_POINT_assoanchorpos;
            Point3 obs_T_point(point.directionVectorFromAssoAnchor( mainAnchor, assoAnchorPose.translation(),
              OBS_T_POINT_point,
              OBS_T_POINT_mainanchorpos,
              OBS_T_POINT_assoanchorpos));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(assoAnchorPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_assoanchorori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_assoanchorori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Matrix PROJ_mainanchorpose = zeros(2,6);
            PROJ_mainanchorpose.block(0,3,2,3) << (PROJ_obs_t_point * OBS_T_POINT_mainanchorpos);
            Dmain->resize(2,6);
            *Dmain << (PROJ_mainanchorpose * MAINANCHORPOSE_mainpose);
          }
          if(Dasso)
          {
            Matrix PROJ_assoanchorpose(2,6);
            PROJ_assoanchorpose.block(0,0,2,3) << PROJ_assoanchorori.block(0,0,2,3);
            PROJ_assoanchorpose.block(0,3,2,3) << (PROJ_obs_t_point * OBS_T_POINT_assoanchorpos);
            Dasso->resize(2,6);
            *Dasso << (PROJ_assoanchorpose * ASSOANCHORPOSE_assopose);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

        }
        else
        {
            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point, OBS_T_POINT_mainpos, OBS_T_POINT_assopos;
            Point3 obs_T_point(point.directionVectorFromAssoAnchor( mainPose.translation(), assoPose.translation(),
              OBS_T_POINT_point,
              OBS_T_POINT_mainpos,
              OBS_T_POINT_assopos));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(assoPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_assoori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_assoori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << zeros(2,3), (PROJ_obs_t_point * OBS_T_POINT_mainpos);
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << PROJ_assoori.block(0,0,2,3), (PROJ_obs_t_point * OBS_T_POINT_assopos);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

      }
    } catch( CheiralityException& e) {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dasso )  *Dasso  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key3()) <<
              " with anchors (" << DefaultKeyFormatter(this->key1()) << "," << DefaultKeyFormatter(this->key2()) << ")" <<
              " moved behind camera " << DefaultKeyFormatter(this->key2()) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return ones(2) * 2.0 * K_->fx();
    }

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };















  /** Non-linear factor for a 2D measurement to a Parallax Angle Point.
   *
   * Non-linear factor for a constraint derived from a 2D measurement bewteen a robot pose and a
   * landmark parametrized as Parallax Angle Point (a.k.a ParallaxAnglePoint3). The measurement
   * comes from from a pose different from the anchors (known as 'other'), and the calibration
   * is known here.
   *
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class ParallaxAngleProjectionFactor: public NoiseModelFactor4<POSE, POSE, POSE, LANDMARK> {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor4<POSE, POSE, POSE, LANDMARK> Base;

    /// shorthand for this class
    typedef ParallaxAngleProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    ParallaxAngleProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of the main anchor
     * @param associatedAnchorKey is the index of the associated anchor
     * @param otherAnchorKey is the index of the other anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    ParallaxAngleProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key associatedAnchorKey, Key otherAnchorKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, associatedAnchorKey, otherAnchorKey, pointKey),
          measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param mainAnchorKey is the index of the main anchor
     * @param associatedAnchorKey is the index of the associated anchor
     * @param otherAnchorKey is the index of the other anchor
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    ParallaxAngleProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key associatedAnchorKey, Key otherAnchorKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, associatedAnchorKey, otherAnchorKey, pointKey),
          measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ParallaxAngleProjectionFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ParallaxAngleProjectionFactor, z = ";
      measured_.print();
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && this->measured_.equals(e->measured_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& mainPose, const Pose3& assoPose, const Pose3& othePose, const ParallaxAnglePoint3& point,
        boost::optional<Matrix&> Dmain  = boost::none,
        boost::optional<Matrix&> Dasso  = boost::none,
        boost::optional<Matrix&> Dothe  = boost::none,
        boost::optional<Matrix&> Dpoint = boost::none) const {
      try {
        // Test if we need jacobians
        if (!Dmain && !Dasso && !Dothe && !Dpoint)
        {
          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Point3 mainAnchor( mainPose.compose(*body_P_sensor_).translation() );
            Point3 assoAnchor( assoPose.compose(*body_P_sensor_).translation() );
            Pose3  camPose   ( othePose.compose(*body_P_sensor_)               );

            // Get the direction to the point from observation point
            Point3 obs_T_point(point.directionVectorFromOtheAnchor( mainAnchor, assoAnchor, camPose.translation()));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(camPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
          else
          {
            // Get the direction to the point from observation point
            Point3 obs_T_point(point.directionVectorFromOtheAnchor( mainPose.translation(), assoPose.translation(), othePose.translation()));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(othePose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Point2 reprojectionError(camera.project(obs_T_point) - measured_);
            return reprojectionError.vector();
          }
        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {
            // Get the main and associated anchors, and the camera pose
            gtsam::Matrix MAINANCHORPOSE_mainpose, ASSOANCHORPOSE_assopose, CAMPOSE_othepose;
            Point3 mainAnchor( mainPose.compose(*body_P_sensor_, MAINANCHORPOSE_mainpose).translation() );
            Point3 assoAnchor( assoPose.compose(*body_P_sensor_, ASSOANCHORPOSE_assopose).translation() );
            Pose3  camPose   ( othePose.compose(*body_P_sensor_, CAMPOSE_othepose       )               );

            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point, OBS_T_POINT_mainanchorpos, OBS_T_POINT_assoanchorpos, OBS_T_POINT_campos;
            Point3 obs_T_point(point.directionVectorFromOtheAnchor( mainAnchor, assoAnchor, camPose.translation(),
              OBS_T_POINT_point,
              OBS_T_POINT_mainanchorpos,
              OBS_T_POINT_assoanchorpos,
              OBS_T_POINT_campos));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(camPose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_camori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_camori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Matrix PROJ_mainanchorpose = zeros(2,6);
            PROJ_mainanchorpose.block(0,3,2,3) << (PROJ_obs_t_point * OBS_T_POINT_mainanchorpos);
            Dmain->resize(2,6);
            *Dmain << (PROJ_mainanchorpose * MAINANCHORPOSE_mainpose);
          }
          if(Dasso)
          {
            Matrix PROJ_assoanchorpose = zeros(2,6);
            PROJ_assoanchorpose.block(0,3,2,3) << (PROJ_obs_t_point * OBS_T_POINT_assoanchorpos);
            Dasso->resize(2,6);
            *Dasso << (PROJ_assoanchorpose * ASSOANCHORPOSE_assopose);
          }
          if(Dothe)
          {
            Matrix PROJ_campose(2,6);
            PROJ_campose << PROJ_camori.block(0,0,2,3), (PROJ_obs_t_point * OBS_T_POINT_campos);
            Dothe->resize(2,6);
            *Dothe << (PROJ_campose * CAMPOSE_othepose);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

        }
        else
        {
            // Get the direction to the point from observation point
            Matrix OBS_T_POINT_point, OBS_T_POINT_mainanchorpos, OBS_T_POINT_assoanchorpos, OBS_T_POINT_campos;
            Point3 obs_T_point(point.directionVectorFromOtheAnchor(
              mainPose.translation(), assoPose.translation(), othePose.translation(),
              OBS_T_POINT_point,
              OBS_T_POINT_mainanchorpos,
              OBS_T_POINT_assoanchorpos,
              OBS_T_POINT_campos));

            // Put a camera at the origin
            PinholeCamera<CALIBRATION> camera(Pose3(othePose.rotation(), Point3()), *K_);

            // Project direction vector to camera and calculate the error
            Matrix PROJ_camori, PROJ_obs_t_point;
            Point2 reprojectionError(camera.project(obs_T_point, PROJ_camori, PROJ_obs_t_point) - measured_);

            // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << zeros(2,3), (PROJ_obs_t_point * OBS_T_POINT_mainanchorpos);
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << zeros(2,3), (PROJ_obs_t_point * OBS_T_POINT_assoanchorpos);
          }
          if(Dothe)
          {
            Dothe->resize(2,6);
            *Dothe << PROJ_camori.block(0,0,2,3), (PROJ_obs_t_point * OBS_T_POINT_campos);
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << (PROJ_obs_t_point * OBS_T_POINT_point);
          }

          return reprojectionError.vector();

      }
    }catch( CheiralityException& e) {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dasso )  *Dasso  = zeros(2,6);
        if (Dothe )  *Dothe  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key4()) <<
              " with anchors (" << DefaultKeyFormatter(this->key1()) << "," << DefaultKeyFormatter(this->key2()) << ")" <<
              " moved behind camera " << DefaultKeyFormatter(this->key3()) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return ones(2) * 2.0 * K_->fx();
    }

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };

} // \ namespace gtsam
