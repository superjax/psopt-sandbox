#include <fstream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "move_box.h"

void WaypointConstraint(adouble* e, adouble* initial_states, adouble* final_states,
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

TEST(DISABLED_MoveBox, MultiWindow)
{
  Alg  algorithm;
  Sol  solution;
  Prob problem;

  DMatrix state_max;
  state_max = "[4.0, 4.0]";
  DMatrix state_min;
  state_min = "[-4.0, -4.0]";
  double control_max = 7.0;
  double control_min = -7.0;

  DMatrix waypoints;
  waypoints = "[0.0, 0.0;"
               "1.0, 4.0;"
               "2.0, 0.0]";
  DMatrix times;
  times = "[0.0, 0.0;"
           "1.0, 1.0;"
           "2.0, 2.0]";
  int nnodes = 50;
  int npath = 0;
  int nevents = 4;
  int ncontrols = 1;
  int nstates = 2;

  problem.nphases = 2;
  problem.nlinkages = 4;
  psopt_level1_setup(problem);

  for (int i = 1; i < waypoints.GetNoRows(); i++)
  {
    problem.phases(i).nodes = nnodes;
    problem.phases(i).npath = npath;
    problem.phases(i).nevents = nevents;
    problem.phases(i).ncontrols = ncontrols;
    problem.phases(i).nstates = nstates;
  }
  psopt_level2_setup(problem, algorithm);

  for (int i = 1; i < waypoints.GetNoRows(); i++)
  {
    problem.phases(i).bounds.lower.controls(1) = control_min;
    problem.phases(i).bounds.upper.controls(1) = control_max;
    problem.phases(i).bounds.lower.states = state_min;
    problem.phases(i).bounds.upper.states = state_max;
    problem.phases(i).bounds.lower.events = waypoints(i, colon()) || waypoints(i+1, colon());
    problem.phases(i).bounds.upper.events = waypoints(i, colon()) || waypoints(i+1, colon());
    problem.phases(i).bounds.lower.StartTime = times(i, 1);
    problem.phases(i).bounds.upper.StartTime = times(i, 2);
    problem.phases(i).bounds.lower.EndTime = times(i+1, i);
    problem.phases(i).bounds.upper.EndTime = times(i+1, i);
    problem.phases(i).nobserved = 0;

    problem.phases(i).guess.controls = zeros(1, nnodes);

    DMatrix x_guess = zeros(2, nnodes);
    x_guess(1,colon()) = linspace(waypoints(i,1), waypoints(i+1,1), nnodes);
    x_guess(1,colon()) = linspace(waypoints(i,2), waypoints(i+1,2), nnodes);
    problem.phases(i).guess.states = zeros(1, nnodes);
    problem.phases(i).guess.time = linspace(i-1, i, nnodes);
  }

  problem.integrand_cost = &MoveBoxCostFunction;
  problem.endpoint_cost	= &emptyEndpoint;
  problem.dae = &MoveBoxDynamicConstraint;
  problem.events = &WaypointConstraint;
  problem.linkages = &emptyLinkage;

  algorithm.nlp_iter_max                = 1000;
  algorithm.nlp_tolerance               = 1.e-6;
  algorithm.nlp_method                  = "IPOPT";
  algorithm.scaling                     = "automatic";
  algorithm.derivatives                 = "automatic";
  algorithm.hessian                     = "exact";
  algorithm.mesh_refinement             = "automatic";
  algorithm.ode_tolerance               = 1.e-5;

  psopt(solution, problem, algorithm);

  DMatrix x = solution.get_states_in_phase(1);
  DMatrix u = solution.get_controls_in_phase(1);
  DMatrix t = solution.get_time_in_phase(1);

  for(int i = 2; i <= problem.nphases; i++)
  {
    DMatrix xi = solution.get_states_in_phase(i);
    DMatrix ui = solution.get_controls_in_phase(i);
    DMatrix ti = solution.get_time_in_phase(i);

    x = x || xi;
    u = u || ui;
    t = t || ti;
  }

  plot(t,x(1,colon()),problem.name+": state", "time (s)", "state", "x1");
  plot(t,x(2,colon()),problem.name+": state", "time (s)", "state", "x2");
  plot(t,u(1,colon()),problem.name+": control","time (s)", "control", "u1");
}


