#pragma once

#include "psopt.h"

class EmptyEndpointCost : public EndpointFunctor
{
public:
  adouble operator()(adouble* initial_states, adouble* final_states,
                     adouble* parameters,adouble& t0, adouble& tf,
                     adouble* xad, int iphase, Workspace* workspace)
  {
    return 0;
  }
};

class EmptyLinkage : public LinkageFunctor
{
public:
  void operator()( adouble* linkages, adouble* xad, Workspace* workspace)
  {
    return;
  }
};
