#include <fstream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "move_box.h"

TEST(DISABLED_MoveBox, SingleWindow)
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

  problem.integrand_cost = new MoveBoxCost();
  problem.endpoint_cost	= new EmptyEndpointCost();
  problem.dae = new BlockDynamics();
  problem.events = new MoveBoxEndpointConstraints();
  problem.linkages = new EmptyLinkage();

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
  algorithm.print_level                 = 0;

  psopt(solution, problem, algorithm);


  DMatrix x = solution.get_states_in_phase(1);
  DMatrix u = solution.get_controls_in_phase(1);
  DMatrix t = solution.get_time_in_phase(1);

  plot(t,x(1,colon()),problem.name+": state", "time (s)", "state", "x1");
  plot(t,x(2,colon()),problem.name+": state", "time (s)", "state", "x2");
  plot(t,u(1,colon()),problem.name+": control","time (s)", "control", "u1");
}


