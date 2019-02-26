#include <fstream>
#include <gtest/gtest.h>

#include "angle_quadrotor.h"

static void log_solution(Sol& solution, string prefix)
{
  std::ofstream x0_file(prefix + "x.dat");
  std::ofstream t_file(prefix + "t.dat");
  std::ofstream u_file(prefix + "u.dat");
  DMatrix x0 = solution.get_states_in_phase(1);
  DMatrix t0 = solution.get_time_in_phase(1);
  DMatrix u0 = solution.get_controls_in_phase(1);
  x0_file.write((char*)x0.GetPr(), sizeof(double)*x0.GetNoCols()*x0.GetNoRows());
  t_file.write((char*)t0.GetPr(), sizeof(double)*t0.GetNoCols()*t0.GetNoRows());
  u_file.write((char*)u0.GetPr(), sizeof(double)*u0.GetNoCols()*u0.GetNoRows());
  x0_file.close();
  t_file.close();
  u_file.close();
}

static void log_problem(Prob& problem, string prefix)
{
  std::ofstream x0_file(prefix + "x.dat");
  std::ofstream t_file(prefix + "t.dat");
  std::ofstream u_file(prefix + "u.dat");
  DMatrix x0 = problem.phases(1).guess.states;
  DMatrix t0 = problem.phases(1).guess.time;
  DMatrix u0 = problem.phases(1).guess.controls;
  x0_file.write((char*)x0.GetPr(), sizeof(double)*x0.GetNoCols()*x0.GetNoRows());
  t_file.write((char*)t0.GetPr(), sizeof(double)*t0.GetNoCols()*t0.GetNoRows());
  u_file.write((char*)u0.GetPr(), sizeof(double)*u0.GetNoCols()*u0.GetNoRows());
  x0_file.close();
  t_file.close();
  u_file.close();
}

TEST(DISABLED_Angle_Quadrotor, SingleWindow)
{
  Alg  algorithm;
  Sol  solution;
  Prob problem;

  DMatrix state_max;
  DMatrix state_min;
  state_max = "[4.0, 4.0, 4.0,"
               "4.0, 4.0, 4.0]";
  state_min = "[-4.0, -4.0, -4.0,"
               "-4.0, -4.0, -4.0]";
  DMatrix control_max;
  DMatrix control_min;
  control_max = "[1.0, 1.0, 1.0, 1.0, 1.0]";
  control_min = "[-1.0, -1.0, -1.0, -1.0, 0.0]";

  DMatrix waypoints;
  waypoints = "[0.0, 0.0, 0.0, 0.0;"
               "0.0, 0.0, -0.2, 0.0]";

  DMatrix x0;
  x0 = "[0.0, 0.0, 0.0,"
        "0.0, 0.0, 0.0]";
  DMatrix times;
  times = "[0.0, 0.0;"
           "3.0, 3.0]";
  int nnodes = 100;
  int npath = 4;
  int nevents = 8;
  int ncontrols = 5;
  int nstates = 6;
  double hover_throttle = 0.5;

  problem.nphases = 1;
  problem.nlinkages = 0;
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
    problem.phases(i).bounds.lower.controls = control_min;
    problem.phases(i).bounds.upper.controls = control_max;
    problem.phases(i).bounds.lower.states = state_min;
    problem.phases(i).bounds.upper.states = state_max;
    problem.phases(i).bounds.lower.events = waypoints(i, colon()) || waypoints(i+1, colon());
    problem.phases(i).bounds.upper.events = waypoints(i, colon()) || waypoints(i+1, colon());;
    problem.phases(i).bounds.lower.StartTime = times(i, 1);
    problem.phases(i).bounds.upper.StartTime = times(i, 2);
    problem.phases(i).bounds.lower.EndTime = times(i+1, i);
    problem.phases(i).bounds.upper.EndTime = times(i+1, i);
    problem.phases(i).nobserved = 0;


    DMatrix x_guess = zeros(nstates, nnodes);
    x_guess(1,colon()) = linspace(waypoints(i,1), waypoints(i+1,1), nnodes);
    x_guess(2,colon()) = linspace(waypoints(i,2), waypoints(i+1,2), nnodes);
    x_guess(3,colon()) = linspace(waypoints(i,3), waypoints(i+1,3), nnodes);
    x_guess(4,colon()) = 1.0;
    problem.phases(i).guess.states = x_guess;

    DMatrix u_guess = zeros(ncontrols, nnodes);
    u_guess(4,colon()) = hover_throttle;
    problem.phases(i).guess.controls = u_guess;

    problem.phases(i).guess.time = linspace(times(i, i), times(i+1, i), nnodes);
  }

  problem.integrand_cost = new AngleQuadrotorCost(hover_throttle);
  problem.endpoint_cost	= new EmptyEndpointCost();
  problem.dae = new AngleQuadrotorDynamics(hover_throttle);
  problem.events = new AngleQuadrotorEndpointConstraint();
  problem.linkages = new EmptyLinkage();

  algorithm.nlp_iter_max                = 1000;
  algorithm.nlp_tolerance               = 1.e-3;
  algorithm.nlp_method                  = "IPOPT";
  algorithm.scaling                     = "automatic";
  algorithm.derivatives                 = "automatic";
  algorithm.mr_max_iterations           = 1;
//  algorithm.jac_sparsity_ratio          = 0.20;
  algorithm.collocation_method          = "Legendre";
  algorithm.diff_matrix                 = "central-differences";
  algorithm.mesh_refinement             = "automatic";
//  algorithm.mr_max_increment_factor     = 0.3;
  algorithm.defect_scaling              = "jacobian-based";
  algorithm.print_level                 = 1;

  log_problem(problem, "quadrotor_init_");
  psopt(solution, problem, algorithm);
  log_solution(solution, "quadrotor_final_");
}

