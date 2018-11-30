#include "quadrotor.h"

#include "AdolcForward.h"
#include "geometry/support.h"
#include "geometry/quat.h"


adouble quadrotorCostFunction(adouble* states, adouble* controls,
                     adouble* parameters, adouble& time, adouble* xad,
                     int iphase, Workspace* workspace)

{
  typedef Eigen::Matrix<adouble, 4, 1> Vec4;
  Eigen::Map<Vec4> u(controls);

  return u.squaredNorm();
}

void quadrotorDynamicConstraint(adouble* derivatives, adouble* path, adouble* states,
                       adouble* controls, adouble* parameters, adouble& time,
                       adouble* xad, int iphase, Workspace* workspace)
{
  typedef Eigen::Matrix<adouble, 3, 1> Vec3;
  typedef Eigen::Matrix<adouble, 4, 1> Vec4;

  Eigen::Map<Vec3> p(states);
  quat::Quat<adouble> q(states+3);
  Eigen::Map<Vec3> v(states+7);

  Eigen::Map<Vec3> w(controls);
  adouble& F(*(controls+3));

  Eigen::Map<Vec3> pdot(derivatives);
  Eigen::Map<Vec4> qdot(derivatives+3);
  Eigen::Map<Vec3> vdot(derivatives+7);

  static const Vec3 g = (Vec3() << 0, 0, 9.80665).finished();
  Vec3 Fvec = (Vec3() << 0, 0, F).finished();

  pdot = q.R().transpose() * v;
  qdot << 1.0, w;
  vdot = skew(v)*w + q.R()*g + Fvec;
}

void EmptyEvent(adouble* e, adouble* initial_states,
                adouble* final_states, adouble* parameters,adouble& t0,
                adouble& tf, adouble* xad, int iphase, Workspace* workspace)

{}

adouble quadrotorWaypointCost(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
  typedef Eigen::Matrix<adouble, 3, 1> Vec3;
  typedef Eigen::Matrix<adouble, 4, 1> Vec4;
  Eigen::Map<Vec3> p0(initial_states);
  Eigen::Map<Vec4> q0(initial_states+3);
  Eigen::Map<Vec3> v0(initial_states+7);
  Eigen::Map<Vec3> pf(final_states);
  Eigen::Map<Vec4> qf(final_states+3);
  Eigen::Map<Vec3> vf(final_states+7);

  Eigen::Map<Vec3> p0_c(e);
  adouble& v0_c(*(e+3));
  Eigen::Map<Vec3> pf_c(e+4);
  Eigen::Map<Vec3> pf_c(e+4);


}




