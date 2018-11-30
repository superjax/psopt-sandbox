#pragma once

#include "psopt.h"

adouble emptyEndpoint(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace);

void emptyLinkage(adouble* emptyLinkage, adouble* xad, Workspace* workspace);

adouble CostFunction(adouble* states, adouble* controls,
                       adouble* parameters, adouble& time, adouble* xad,
                       int iphase, Workspace* workspace);

void dynamicConstraint(adouble* derivatives, adouble* path, adouble* states,
         adouble* controls, adouble* parameters, adouble& time,
         adouble* xad, int iphase, Workspace* workspace);
