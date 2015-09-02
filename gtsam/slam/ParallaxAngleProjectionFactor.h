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
  class ParallaxAngleMainAnchorProjectionFactor: public NoiseModelFactor2<POSE, LANDMARK>
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
    typedef ParallaxAngleMainAnchorProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    ParallaxAngleMainAnchorProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

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
    ParallaxAngleMainAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
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
    ParallaxAngleMainAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, pointKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ParallaxAngleMainAnchorProjectionFactor() {}

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
      std::cout << s << "ParallaxAngleMainAnchorProjectionFactor, z = ";
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
          // Get the direction to the point from observation point
          Point3 dirMainToPoint( point.directionVector() );

          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_) );

            Point3 pc = mainCameraPose.rotation().unrotate(dirMainToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();

          }
          else
          {

            Point3 pc = mainPose.rotation().unrotate(dirMainToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();
          }

        }

        // Same computation but with jacobians

        // Get the direction to the point from observation point
        Matrix DIRMAINTOPOINT_point;
        Point3 dirMainToPoint(point.directionVector( DIRMAINTOPOINT_point ));

        if(body_P_sensor_)
        {

          // Get the main and associated anchors, and the camera pose
          Matrix MAINCAMERAPOSE_mainpose;
          Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_, MAINCAMERAPOSE_mainpose) );

          Matrix PC_maincamerarot, PC_dirmaintopoint;
          Point3 pc = mainCameraPose.rotation().unrotate(dirMainToPoint, PC_maincamerarot, PC_dirmaintopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_maincamerapose = Matrix::Zero(3, 6);
          PC_maincamerapose.block(0, 0, 3, 3) = PC_maincamerarot;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          Point2 projection = K_->uncalibrate(pn);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pn * PN_pc * PC_maincamerapose * MAINCAMERAPOSE_mainpose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,2);
            *Dpoint << PI_pn * PN_pc * PC_dirmaintopoint * DIRMAINTOPOINT_point;
          }

          return reprojectionError.vector();

        }
        else
        {

          Matrix PC_mainrot, PC_dirmaintopoint;
          Point3 pc = mainPose.rotation().unrotate(dirMainToPoint, PC_mainrot, PC_dirmaintopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_mainpose = Matrix::Zero(3, 6);
          PC_mainpose.block(0, 0, 3, 3) = PC_mainrot;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          Point2 projection = K_->uncalibrate(pn);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pn * PN_pc * PC_mainpose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,2);
            *Dpoint << PI_pn * PN_pc * PC_dirmaintopoint * DIRMAINTOPOINT_point;
          }

          return reprojectionError.vector();

        }
      } catch( CheiralityException& e)
      {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,2);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark " << DefaultKeyFormatter(this->key2()) <<
            " moved behind main anchor camera " << DefaultKeyFormatter(this->key1()) << std::endl;
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
          // Get the direction to the point from observation point
          Point3 dirMainToPoint( point.directionVectorFromMainAnchor() );

          if(body_P_sensor_)
          {
            // Get the main and associated anchors, and the camera pose
            Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_) );

            Point3 pc = mainCameraPose.rotation().unrotate(dirMainToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();

          }
          else
          {

            Point3 pc = mainPose.rotation().unrotate(dirMainToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();
          }

        }

        // Same computation but with jacobians

        // Get the direction to the point from observation point
        Matrix DIRMAINTOPOINT_point(3,3);
        Point3 dirMainToPoint(point.directionVectorFromMainAnchor( DIRMAINTOPOINT_point ));

        if(body_P_sensor_)
        {

          // Get the main and associated anchors, and the camera pose
          Matrix MAINCAMERAPOSE_mainpose;
          Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_, MAINCAMERAPOSE_mainpose) );

          Matrix PC_maincamerarot, PC_dirmaintopoint;
          Point3 pc = mainCameraPose.rotation().unrotate(dirMainToPoint, PC_maincamerarot, PC_dirmaintopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_maincamerapose = Matrix::Zero(3, 6);
          PC_maincamerapose.block(0, 0, 3, 3) = PC_maincamerarot;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pn * PN_pc * PC_maincamerapose * MAINCAMERAPOSE_mainpose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pn * PN_pc * PC_dirmaintopoint * DIRMAINTOPOINT_point;
          }

          return reprojectionError.vector();

        }
        else
        {

          Matrix PC_mainrot, PC_dirmaintopoint;
          Point3 pc = mainPose.rotation().unrotate(dirMainToPoint, PC_mainrot, PC_dirmaintopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_mainpose = Matrix::Zero(3, 6);
          PC_mainpose.block(0, 0, 3, 3) = PC_mainrot;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pn * PN_pc * PC_mainpose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pn * PN_pc * PC_dirmaintopoint * DIRMAINTOPOINT_point;
          }

          return reprojectionError.vector();

        }
      } catch( CheiralityException& e)
      {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark " << DefaultKeyFormatter(this->key2()) <<
            " moved behind main anchor camera " << DefaultKeyFormatter(this->key1()) << std::endl;
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
  class ParallaxAngleAssoAnchorProjectionFactor: public NoiseModelFactor3<POSE, POSE, LANDMARK> {
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
    typedef ParallaxAngleAssoAnchorProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    ParallaxAngleAssoAnchorProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

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
    ParallaxAngleAssoAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
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
    ParallaxAngleAssoAnchorProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key mainAnchorKey, Key associatedAnchorKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, mainAnchorKey, associatedAnchorKey, pointKey),
          measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ParallaxAngleAssoAnchorProjectionFactor() {}

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
      std::cout << s << "ParallaxAngleAssoAnchorProjectionFactor, z = ";
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
            // Get the main and associated camera poses
            Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_) );
            Pose3 assoCameraPose ( assoPose.compose(*body_P_sensor_) );

            // Get the direction to the point from observation point
            Point3 dirAssoToPoint( point.directionVectorFromAssoAnchor( mainCameraPose.translation(), assoCameraPose.translation() ) );

            Point3 pc = assoCameraPose.rotation().unrotate(dirAssoToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();

          }
          else
          {
            // Get the direction to the point from observation point
            Point3 dirAssoToPoint( point.directionVectorFromAssoAnchor( mainPose.translation(), assoPose.translation() ) );

            Point3 pc = assoPose.rotation().unrotate(dirAssoToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();
          }

        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {

          // Get the main and associated camera poses
          Matrix MAINCAMERAPOSE_mainpose;
          Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_, MAINCAMERAPOSE_mainpose) );
          Matrix ASSOCAMERAPOSE_assopose;
          Pose3 assoCameraPose ( assoPose.compose(*body_P_sensor_, ASSOCAMERAPOSE_assopose) );

          // Get the direction to the point from observation point
          Matrix DIRASSOTOPOINT_point, DIRASSOTOPOINT_maincameratr, DIRASSOTOPOINT_assocameratr;
          Point3 dirAssoToPoint( point.directionVectorFromAssoAnchor( mainCameraPose.translation(), assoCameraPose.translation(), DIRASSOTOPOINT_point, DIRASSOTOPOINT_maincameratr, DIRASSOTOPOINT_assocameratr) );

          Matrix DIRASSOTOPOINT_maincamerapose = Matrix::Zero(3, 6);
          DIRASSOTOPOINT_maincamerapose.block(0, 3, 3, 3) = DIRASSOTOPOINT_maincameratr;

          Matrix PC_assocamerarot, PC_dirassotopoint;
          Point3 pc = assoCameraPose.rotation().unrotate(dirAssoToPoint, PC_assocamerarot, PC_dirassotopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_assocamerapose = Matrix::Zero(3, 6);
          PC_assocamerapose << PC_assocamerarot, PC_dirassotopoint*DIRASSOTOPOINT_assocameratr;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          Matrix PI_pc = PI_pn * PN_pc;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pc * PC_dirassotopoint * DIRASSOTOPOINT_maincamerapose * MAINCAMERAPOSE_mainpose;
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << PI_pc * PC_assocamerapose * ASSOCAMERAPOSE_assopose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pc * PC_dirassotopoint * DIRASSOTOPOINT_point;
          }

          return reprojectionError.vector();

        }
        else
        {

          // Get the direction to the point from observation point
          Matrix DIRASSOTOPOINT_point, DIRASSOTOPOINT_maintr, DIRASSOTOPOINT_assotr;
          Point3 dirAssoToPoint( point.directionVectorFromAssoAnchor( mainPose.translation(), assoPose.translation(), DIRASSOTOPOINT_point, DIRASSOTOPOINT_maintr, DIRASSOTOPOINT_assotr) );

          Matrix DIRASSOTOPOINT_mainpose = Matrix::Zero(3, 6);
          DIRASSOTOPOINT_mainpose.block(0, 3, 3, 3) = DIRASSOTOPOINT_maintr;

          Matrix PC_assorot, PC_dirassotopoint;
          Point3 pc = assoPose.rotation().unrotate(dirAssoToPoint, PC_assorot, PC_dirassotopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_assopose = Matrix::Zero(3, 6);
          PC_assopose << PC_assorot, PC_dirassotopoint*DIRASSOTOPOINT_assotr;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          Matrix PI_pc = PI_pn * PN_pc;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pc * PC_dirassotopoint * DIRASSOTOPOINT_mainpose;
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << PI_pc * PC_assopose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pc * PC_dirassotopoint * DIRASSOTOPOINT_point;
          }

          return reprojectionError.vector();

      }
    } catch( CheiralityException& e) {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dasso )  *Dasso  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark " << DefaultKeyFormatter(this->key3())
                    << " with main anchor " << DefaultKeyFormatter(this->key1())
                    << " moved behind associated camera anchor " << DefaultKeyFormatter(this->key2())
                    << std::endl;
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
            // Get the main and associated camera poses
            Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_) );
            Pose3 assoCameraPose ( assoPose.compose(*body_P_sensor_) );
            Pose3 otheCameraPose ( othePose.compose(*body_P_sensor_) );

            // Get the direction to the point from observation point
            Point3 dirOtheToPoint( point.directionVectorFromOtheAnchor( mainCameraPose.translation(), assoCameraPose.translation(), otheCameraPose.translation() ) );

            Point3 pc = otheCameraPose.rotation().unrotate(dirOtheToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();

          }
          else
          {

            // Get the direction to the point from observation point
            Point3 dirOtheToPoint( point.directionVectorFromOtheAnchor( mainPose.translation(), assoPose.translation(), othePose.translation() ) );

            Point3 pc = othePose.rotation().unrotate(dirOtheToPoint); // get direction in camera frame (translation does not matter)
            Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc); // project the point to the camera
            Point2 pi = K_->uncalibrate(pn);
            Point2 reprojectionError = pi - measured_;

            return reprojectionError.vector();

          }

        }

        // Same computation but with jacobians
        if(body_P_sensor_)
        {

          // Get the main and associated camera poses
          Matrix MAINCAMERAPOSE_mainpose;
          Pose3 mainCameraPose ( mainPose.compose(*body_P_sensor_, MAINCAMERAPOSE_mainpose) );
          Matrix ASSOCAMERAPOSE_assopose;
          Pose3 assoCameraPose ( assoPose.compose(*body_P_sensor_, ASSOCAMERAPOSE_assopose) );
          Matrix OTHECAMERAPOSE_othepose;
          Pose3 otheCameraPose ( othePose.compose(*body_P_sensor_, OTHECAMERAPOSE_othepose) );

          // Get the direction to the point from observation point
          Matrix DIROTHETOPOINT_point, DIROTHETOPOINT_maincameratr, DIROTHETOPOINT_assocameratr, DIROTHETOPOINT_othecameratr;
          Point3 dirOtheToPoint( point.directionVectorFromOtheAnchor( mainCameraPose.translation(), assoCameraPose.translation(), otheCameraPose.translation(), DIROTHETOPOINT_point, DIROTHETOPOINT_maincameratr, DIROTHETOPOINT_assocameratr, DIROTHETOPOINT_othecameratr) );

          Matrix DIROTHETOPOINT_maincamerapose = Matrix::Zero(3, 6);
          DIROTHETOPOINT_maincamerapose.block(0, 3, 3, 3) = DIROTHETOPOINT_maincameratr;
          Matrix DIROTHETOPOINT_assocamerapose = Matrix::Zero(3, 6);
          DIROTHETOPOINT_assocamerapose.block(0, 3, 3, 3) = DIROTHETOPOINT_assocameratr;

          Matrix PC_othecamerarot, PC_dirothetopoint;
          Point3 pc = otheCameraPose.rotation().unrotate(dirOtheToPoint, PC_othecamerarot, PC_dirothetopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_othecamerapose = Matrix::Zero(3, 6);
          PC_othecamerapose << PC_othecamerarot, PC_dirothetopoint*DIROTHETOPOINT_othecameratr;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          Matrix PI_pc = PI_pn * PN_pc;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_maincamerapose * MAINCAMERAPOSE_mainpose;
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_assocamerapose * ASSOCAMERAPOSE_assopose;
          }
          if(Dothe)
          {
            Dothe->resize(2,6);
            *Dothe << PI_pc * PC_othecamerapose * OTHECAMERAPOSE_othepose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_point;
          }

          return reprojectionError.vector();

        }
        else
        {

          // Get the direction to the point from observation point
          Matrix DIROTHETOPOINT_point, DIROTHETOPOINT_maintr, DIROTHETOPOINT_assotr, DIROTHETOPOINT_othetr;
          Point3 dirOtheToPoint( point.directionVectorFromOtheAnchor( mainPose.translation(), assoPose.translation(), othePose.translation(), DIROTHETOPOINT_point, DIROTHETOPOINT_maintr, DIROTHETOPOINT_assotr, DIROTHETOPOINT_othetr) );

          Matrix DIROTHETOPOINT_mainpose = Matrix::Zero(3, 6);
          DIROTHETOPOINT_mainpose.block(0, 3, 3, 3) = DIROTHETOPOINT_maintr;
          Matrix DIROTHETOPOINT_assopose = Matrix::Zero(3, 6);
          DIROTHETOPOINT_assopose.block(0, 3, 3, 3) = DIROTHETOPOINT_assotr;

          Matrix PC_otherot, PC_dirothetopoint;
          Point3 pc = othePose.rotation().unrotate(dirOtheToPoint, PC_otherot, PC_dirothetopoint); // get direction in camera frame (translation does not matter)

          Matrix PC_othepose = Matrix::Zero(3, 6);
          PC_othepose << PC_otherot, PC_dirothetopoint*DIROTHETOPOINT_othetr;

          Matrix PN_pc; // 2*3
          Point2 pn = PinholeCamera<CALIBRATION>::project_to_camera(pc, PN_pc);

          // uncalibration
          Matrix PI_pn; // 2*2
          Point2 pi = K_->uncalibrate(pn, boost::none, PI_pn);

          Point2 reprojectionError = pi - measured_;

          Matrix PI_pc = PI_pn * PN_pc;

          // Chain of jacobians
          if(Dmain)
          {
            Dmain->resize(2,6);
            *Dmain << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_mainpose;
          }
          if(Dasso)
          {
            Dasso->resize(2,6);
            *Dasso << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_assopose;
          }
          if(Dothe)
          {
            Dothe->resize(2,6);
            *Dothe << PI_pc * PC_othepose;
          }
          if(Dpoint)
          {
            Dpoint->resize(2,3);
            *Dpoint << PI_pc * PC_dirothetopoint * DIROTHETOPOINT_point;
          }

          return reprojectionError.vector();

      }
    }catch( CheiralityException& e) {
        if (Dmain )  *Dmain  = zeros(2,6);
        if (Dasso )  *Dasso  = zeros(2,6);
        if (Dothe )  *Dothe  = zeros(2,6);
        if (Dpoint)  *Dpoint = zeros(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark " << DefaultKeyFormatter(this->key4())
                    << " with main anchor " << DefaultKeyFormatter(this->key1())
                    << " and associated anchor " << DefaultKeyFormatter(this->key2())
                    << " moved behind camera " << DefaultKeyFormatter(this->key3())
                    << std::endl;
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
