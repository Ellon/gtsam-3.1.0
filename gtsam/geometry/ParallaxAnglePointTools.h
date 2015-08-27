#pragma once

#include <gtsam/base/Matrix.h>

namespace gtsam {

    gtsam::Vector2 vec2py(gtsam::Vector3 vec, boost::optional<gtsam::Matrix&> PY_vec = boost::none);

    gtsam::Vector3 py2vec(gtsam::Vector2 py, boost::optional<gtsam::Matrix&> VEC_py = boost::none);

    gtsam::Vector3 py2vec(double pitch, double yaw,
                          boost::optional<gtsam::Matrix&> VEC_pitch = boost::none,
                          boost::optional<gtsam::Matrix&> VEC_yaw = boost::none);

    double norm(gtsam::Vector3 v, boost::optional<gtsam::Matrix&> NORMV_v = boost::none);

    double dot(gtsam::Vector3 v1, gtsam::Vector3 v2,
               boost::optional<gtsam::Matrix&> DOT_v1 = boost::none,
               boost::optional<gtsam::Matrix&> DOT_v2 = boost::none);

    double vectors2angle( const gtsam::Vector3 & v1, const gtsam::Vector3 & v2,
                          boost::optional<gtsam::Matrix&> ANG_v1 = boost::none,
                          boost::optional<gtsam::Matrix&> ANG_v2 = boost::none);

}