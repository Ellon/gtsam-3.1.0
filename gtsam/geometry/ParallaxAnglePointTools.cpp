
#include "ParallaxAnglePointTools.h"

#include <gtsam/geometry/Point3.h>

using namespace std;

namespace gtsam {

Vector2 vec2py(Vector3 vec, boost::optional<gtsam::Matrix&> PY_vec)
{

  double x = vec(0), y = vec(1), z = vec(2);

  double xy2  = x*x + y*y;
  double rxy = sqrt(xy2);

  double pitch = atan2(z,rxy);
  double yaw   = atan2(y,x);

  if(PY_vec)
  {
    double xyz2 = xy2 + z*z;

    PY_vec->resize(2,3);
    *PY_vec << -z/rxy*x/xyz2, -z/rxy*y/xyz2,  rxy/xyz2,
                  -y/xy2,         x/xy2,         0;
  }

  return Vector2(pitch, yaw);

}

Vector3 py2vec(Vector2 py, boost::optional<gtsam::Matrix&> VEC_py)
{
  if(VEC_py)
  {
    Matrix VEC_p, VEC_y;
    Vector3 vec = py2vec(py(0), py(1), VEC_p, VEC_y);
    VEC_py->resize(3,2);
    *VEC_py << VEC_p, VEC_y;
    return vec;
  }

  return py2vec(py(0), py(1));
}

Vector3 py2vec(double pitch, double yaw,
  boost::optional<gtsam::Matrix&> VEC_pitch,
  boost::optional<gtsam::Matrix&> VEC_yaw)
{
  if(VEC_pitch)
  {
    VEC_pitch->resize(3,1);
    *VEC_pitch << -sin(pitch)*cos(yaw), -sin(pitch)*sin(yaw), cos(pitch);
  }

  if(VEC_yaw)
  {
    VEC_yaw->resize(3,1);
    *VEC_yaw << -cos(pitch)*sin(yaw), cos(pitch)*cos(yaw), 0;
  }

  return Vector3(cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), sin(pitch));
}


// Local Functions
double vectors2angle(
  const gtsam::Vector3 & v1, const gtsam::Vector3 & v2,
  boost::optional<gtsam::Matrix&> ANG_v1,
  boost::optional<gtsam::Matrix&> ANG_v2)
{
  if(!ANG_v1 && !ANG_v2)
  {
    double aux = v1.dot(v2)/(v1.norm()*v2.norm());

    // Guard to avoid rounding errors
    if (aux < -1.0) aux = -1.0 ;
    else if (aux > 1.0) aux = 1.0 ;

    return acos( aux );
  }

  Matrix V1NORMED_v1, V2NORMED_v2;
  Vector v1normed = Point3(v1).normalize(V1NORMED_v1).vector();
  Vector v2normed = Point3(v2).normalize(V2NORMED_v2).vector();

  Matrix DOTPROD_v1normed(1,3), DOTPROD_v2normed(1,3);
  double dotprod = dot(v1normed, v2normed, DOTPROD_v1normed, DOTPROD_v2normed);

  Matrix DOTPROD_v1 = DOTPROD_v1normed * V1NORMED_v1;
  Matrix DOTPROD_v2 = DOTPROD_v2normed * V2NORMED_v2;

  double ANG_dotprod;

  // Guard against nonlinearity on the derivate of acos
  if(1.0 - dotprod*dotprod < 1e-6)
  {
    ANG_dotprod = -1.0/1e-6;
  }
  else
  {
    ANG_dotprod = -1.0/sqrt(1.0 - dotprod*dotprod);
  }

  if(ANG_v1)
  {
    ANG_v1->resize(1,3);
    *ANG_v1 << ANG_dotprod*DOTPROD_v1;
  }

  if(ANG_v2)
  {
    ANG_v2->resize(1,3);
    *ANG_v2 << ANG_dotprod*DOTPROD_v2;
  }

  // Guard to avoid rounding errors
  if (dotprod < -1.0) dotprod = -1.0 ;
  else if (dotprod > 1.0) dotprod = 1.0 ;

  return acos( dotprod );

}

double dot(gtsam::Vector3 v1, gtsam::Vector3 v2,
           boost::optional<gtsam::Matrix&> DOT_v1,
           boost::optional<gtsam::Matrix&> DOT_v2)
{
  if(DOT_v1)
  {
    DOT_v1->resize(1,3);
    *DOT_v1 << v2.transpose();
  }

  if(DOT_v2)
  {
    DOT_v2->resize(1,3);
    *DOT_v2 << v1.transpose();
  }

  return v1.dot(v2);

}


double norm(gtsam::Vector3 v, boost::optional<gtsam::Matrix&> Dv)
{
  double n = v.norm();
  if(Dv)
  {
    Dv->resize(1,3);
    *Dv << v(0)/n, v(1)/n, v(2)/n;
  }
  return n;
}

}