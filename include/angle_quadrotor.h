#pragma once


#include "psopt.h"
#include "common.h"

#include "AdolcForward.h"
#include "geometry/support.h"
#include "geometry/quat.h"

struct AngleQuadrotorCost : CostFunctor
{
  AngleQuadrotorCost(double hover_throttle)
  {
    hover_throttle_ = hover_throttle;
  }
  adouble operator()(adouble* states, adouble* controls,
                     adouble* parameters, adouble& time, adouble* xad,
                     int iphase, Workspace* workspace) override

  {
//    typedef Eigen::Matrix<adouble, 5, 1> Vec5;
//    Eigen::Map<Vec5> u(controls);

//    return u.squaredNorm();
//    adouble F = controls[4] - hover_throttle_;
//    return F*F;
    return 0;
  }
  double hover_throttle_;
};

struct AngleQuadrotorDynamics: public DaeFunctor
{
  AngleQuadrotorDynamics(double hover_throttle)
  {
    hover_throttle_ = hover_throttle;
  }
  void operator()(adouble* derivatives, adouble* path, adouble* states,
                  adouble* controls, adouble* parameters, adouble& time,
                  adouble* xad, int iphase, Workspace* workspace) override
  {
    typedef Eigen::Matrix<adouble, 3, 1> Vec3;
    typedef Eigen::Matrix<adouble, 4, 1> Vec4;

    Eigen::Map<Vec3> p(states);
    Eigen::Map<Vec3> v(states+3);

    quat::Quat<adouble> q(controls);
    adouble& F(*(controls+4));

    Eigen::Map<Vec3> pdot(derivatives);
    Eigen::Map<Vec3> vdot(derivatives+3);

    static const Vec3 g = (Vec3() << 0, 0, 1.0).finished();
    Vec3 Fvec = (Vec3() << 0, 0, F).finished();

    pdot = q.R().transpose() * v;
    vdot = q.R()*g - Fvec/(adouble)hover_throttle_;

    path[0] = q.arr_.norm() - 1.0;
    path[1] = 1.0  - q.arr_.norm();
    path[2] = q.z();
    path[3] = -q.z();
  }

  double hover_throttle_;
};

struct AngleQuadrotorWaypointCost : public EndpointFunctor
{
  AngleQuadrotorWaypointCost(Eigen::MatrixXd& waypoints)
  {
    waypoints_ = waypoints;
  }
  AngleQuadrotorWaypointCost(DMatrix& waypoints)
  {
    waypoints_ = Eigen::Map<MatrixXd>(waypoints.GetPr(), waypoints.GetNoRows(), waypoints.GetNoCols());
  }
  adouble operator()(adouble* initial_states, adouble* final_states,
                     adouble* parameters,adouble& t0, adouble& tf,
                     adouble* xad, int iphase, Workspace* workspace) override
  {
    typedef Eigen::Matrix<adouble, 3, 1> Vec3;
    typedef Eigen::Matrix<adouble, 4, 1> Vec4;
    Eigen::Map<Vec3> p0(initial_states);
    Eigen::Map<Vec3> v0(initial_states+3);
    Eigen::Map<Vec3> pf(final_states);
    Eigen::Map<Vec3> vf(final_states+3);

    int id = iphase - 1; // because PSOPT uses stupid MATLAB indexing

    Vec3 p0_c = waypoints_.block<1,3>(id, 0).transpose().cast<adouble>();
    adouble v0_c = waypoints_(id, 3);
    Vec3 pf_c = waypoints_.block<1,3>(id+1, 0).transpose().cast<adouble>();
    adouble vf_c = waypoints_(id+1, 3);

    Vec3 dp0 = p0_c - p0;
    Vec3 dpf = pf_c - pf;
    adouble dv0 = v0_c - v0.norm();
    adouble dvf = vf_c - vf.norm();

    adouble L;
    L = 100 * (dp0.squaredNorm() + dpf.squaredNorm() + dv0*dv0 + dvf*dvf);
    return L;
  }
  MatrixXd waypoints_;
};

struct AngleQuadrotorEndpointConstraint : public EventFunctor
{
  void operator()(adouble* e, adouble* initial_states, adouble* final_states,
                  adouble* parameters,adouble& t0, adouble& tf, adouble* xad,
                  int iphase, Workspace* workspace) override
  {
    typedef Eigen::Matrix<adouble, 3, 1> Vec3;
    typedef Eigen::Matrix<adouble, 4, 1> Vec4;
    if (iphase == 1)
    {
      Eigen::Map<Vec3> p0(initial_states);
      Eigen::Map<Vec3> v0(initial_states+3);
      Eigen::Map<Vec3> ep0(e);
      ep0 = p0;
      e[3] = v0.norm();


      Eigen::Map<Vec3> pf(final_states);
      Eigen::Map<Vec3> vf(final_states+3);
      Eigen::Map<Vec3> epf(e+4);
      epf = pf;
      e[7] = vf.norm();
    }
  }
};




