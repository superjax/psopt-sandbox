#include "common.h"

adouble emptyEndpoint(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
   return 0;
}

void emptyLinkage( adouble* emptyLinkage, adouble* xad, Workspace* workspace) {}
