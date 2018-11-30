#include "move_box.h"


adouble emptyEndpoint(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
   return 0;
}

void emptyLinkage( adouble* emptyLinkage, adouble* xad, Workspace* workspace) {}

adouble CostFunction(adouble* states, adouble* controls,
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


void dynamicConstraint(adouble* derivatives, adouble* path, adouble* states,
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
