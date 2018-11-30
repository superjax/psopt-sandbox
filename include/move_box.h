#pragma once

#include "common.h"

class autoLinkage : public LinkageFunctor
{
public:
  autoLinkage(int id_0, int id_1) : id_0_{id_0}, id_1_{id_1} {}

  void operator()(adouble* linkages, adouble* xad, Workspace* workspace)
  {
    int index = 0;
    auto_link(linkages, &index, xad, id_0_+1, id_1_+1, workspace);
  }

  int id_0_;
  int id_1_;
};


class MoveBoxCost : public CostFunctor
{
public:
  MoveBoxCost(){}
  adouble operator()(adouble* states, adouble* controls,
                     adouble* parameters, adouble& time, adouble* xad,
                     int iphase, Workspace* workspace)
  {
    adouble x1 = states[0];
    adouble x2 = states[1];
    adouble u1 = controls[0];

    adouble L;

    L = ( u1*u1 );

    return  L;
  }
};

class BlockDynamics : public DaeFunctor
{
public:
  void  operator()(adouble* derivatives, adouble* path, adouble* states,
                   adouble* controls, adouble* parameters, adouble& time,
                   adouble* xad, int iphase, Workspace* workspace)
  {
    adouble x1 = states[0];
    adouble x2 = states[1];
    adouble u1 = controls[0];
    adouble t  = time;

    derivatives[0] = x2;
    derivatives[1] = u1;
  }
};

class MoveBoxEndpointConstraints : public EventFunctor
{
public:
  void operator()(adouble* e, adouble* initial_states, adouble* final_states,
                  adouble* parameters,adouble& t0, adouble& tf, adouble* xad,
                  int iphase, Workspace* workspace)

  {
    adouble x1i = initial_states[0];
    adouble x2i = initial_states[1];
    adouble x1f = final_states[0];
    adouble x2f = final_states[1];

    e[0] = x1i;
    e[1] = x2i;
    e[2] = x1f;
    e[3] = x2f;
  }
};
