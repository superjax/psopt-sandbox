#include <fstream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "psopt.h"


adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters,adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
   return 0;
}

void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{
  // No linkages as this is a single phase problem
}


adouble integrand_cost(adouble* states, adouble* controls,
                       adouble* parameters, adouble& time, adouble* xad,
                       int iphase, Workspace* workspace)
{
    adouble x1 = states[0];
    adouble x2 = states[1];
    adouble u1 = controls[0];

    adouble L;

    L = ( x1*x1 + x2*x2 ) + ( u1*u1 );

    return  L;
}


void dae(adouble* derivatives, adouble* path, adouble* states,
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

void events(adouble* e, adouble* initial_states, adouble* final_states,
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


TEST(MoveBox, SingleWindow)
{
  Alg  algorithm;
  Sol  solution;
  Prob problem;

  problem.nphases = 1;
  problem.nlinkages = 0;
  psopt_level1_setup(problem);

  problem.phases(1).nstates = 2;
  problem.phases(1).ncontrols = 1;
  problem.phases(1).nevents = 4;
  problem.phases(1).npath = 0;
  problem.phases(1).nodes = "[50]";

  psopt_level2_setup(problem, algorithm);

  problem.phases(1).bounds.lower.states = "[-4.0, -4.0]";
  problem.phases(1).bounds.upper.states = "[ 4.0,  4.0]";
  problem.phases(1).bounds.lower.controls = "[ -500.0 ]";
  problem.phases(1).bounds.upper.controls = "[  500.0 ]";
  problem.phases(1).bounds.lower.events =  "[0.0, 0.0, 1.0, 0.0]";
  problem.phases(1).bounds.upper.events = problem.phases(1).bounds.lower.events;
  problem.phases(1).bounds.lower.StartTime    = 0.0;
  problem.phases(1).bounds.upper.StartTime    = 0.0;
  problem.phases(1).bounds.lower.EndTime      = 1.0;
  problem.phases(1).bounds.upper.EndTime      = 1.0;

  problem.integrand_cost = &integrand_cost;
  problem.endpoint_cost	= &endpoint_cost;
  problem.dae = &dae;
  problem.events = &events;
  problem.linkages = &linkages;

  int nnodes = problem.phases(1).nodes(1);

  DMatrix x_guess =  zeros(2,nnodes);

  x_guess(1,colon()) = linspace(0,1,nnodes);
  x_guess(2,colon()) = linspace(1,1,nnodes);

  problem.phases(1).guess.controls = zeros(2,nnodes);
  problem.phases(1).guess.states = x_guess;
  problem.phases(1).guess.time = linspace(0.0,1.0,nnodes+1);

  algorithm.nlp_iter_max                = 1000;
  algorithm.nlp_tolerance               = 1.e-6;
  algorithm.nlp_method                  = "IPOPT";
  algorithm.scaling                     = "automatic";
  algorithm.derivatives                 = "automatic";
  algorithm.jac_sparsity_ratio          = 0.20;
  algorithm.collocation_method          = "Legendre";
  algorithm.diff_matrix                 = "central-differences";
  algorithm.mesh_refinement             = "automatic";
  algorithm.mr_max_increment_factor     = 0.3;
  algorithm.defect_scaling              = "jacobian-based";

  psopt(solution, problem, algorithm);

  DMatrix x = solution.get_states_in_phase(1);
  DMatrix u = solution.get_controls_in_phase(1);
  DMatrix t = solution.get_time_in_phase(1);

  plot(t,x(1,colon()),problem.name+": state", "time (s)", "state", "x1");
  plot(t,x(2,colon()),problem.name+": state", "time (s)", "state", "x2");
  plot(t,u(1,colon()),problem.name+": control","time (s)", "control", "u1");
}

