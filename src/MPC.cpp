#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iostream>


using CppAD::AD;
using namespace std;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start      = 0;
size_t y_start      = x_start + N;
size_t psi_start    = y_start + N;
size_t v_start      = psi_start + N;
size_t cte_start    = v_start + N;
size_t epsi_start   = cte_start + N;
size_t delta_start  = epsi_start + N;
size_t a_start      = delta_start + N - 1;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
const double ref_v = 40;

// C O S T S  M U L T I P L I E R S
double a0 = 20;
double a1 = 20;
double a2 = 0.2;
double b0 = 9;
double b1 = 11;
double c0 = 12;
double c1 = 1;

class FG_eval {

  // std::cout <<"[a0]"<<a0<<"[a1]"<<a1<<"[a2]"<<a2
  //           <<"[b0]"<<b0<<"[b1]"<<b1
  //           <<"[c0]"<<c0<<"[c1]"<<c1<<std::endl;

 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // `fg` a vector of the cost constraints,
    // `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // C O S T S

    // cost based on reference state
    for (int t = 0; t < N; t++) {
      fg[0] += a0 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += a1 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += a2 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += b0 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += b1 * CppAD::pow(vars[a_start + t], 2);
    }
    // cost to reduce huge changes in actuations
    for (int t = 0; t < N - 2; t++) {
      fg[0] += c0 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += c1 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


    // M O D E L  C O N S T R A I N T S

    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // note that Ipopt expects all the constraints and variables as vectors.
    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {

      AD<double> x1   = vars[x_start + t];
      AD<double> y1   = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1   = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1= vars[epsi_start + t];

      AD<double> x0   = vars[x_start + t - 1];
      AD<double> y0   = vars[y_start + t -1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0   = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0= vars[epsi_start + t - 1];

      AD<double> d0   = vars[delta_start + t - 1];
      AD<double> a0   = vars[a_start + t - 1];

      // reference y value according to our polynomial
      AD<double> f0   = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0
                        + coeffs[3]*x0*x0*x0;
      // desired orientation value according to the line tangent at x0
      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0
                                       + coeffs[1]);

      // we actually don't/can't compute the constraint at first N point
      // setup the rest of the model constraints
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t]   = psi1 - (psi0 - v0 * d0 / Lf * dt);
      fg[1 + v_start + t]     = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t]   = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t]  = epsi1 - ((psi0 - psides0) - v0 * d0/Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // extract state variables for readability
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // number of independent variables. i.e., vars that can be changed by ipopt.
  // We have 6 state variables over N timesteps
  // We have 2 actuators over N-1 timesteps (b/c no controls on last timestep)
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHALL BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start]     = x;
  vars[y_start]     = y;
  vars[psi_start]   = psi;
  vars[v_start]     = v;
  vars[cte_start]   = cte;
  vars[epsi_start]  = epsi;


  // L I M I T S  -- aka --  N O N H O L O M O N I C

  // placeholders for the upper and lower limits of our state variables and
  // actuators
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] =  0.436332;
  }
  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // assign the given initial state variables
  constraints_lowerbound[x_start]     = x;
  constraints_lowerbound[y_start]     = y;
  constraints_lowerbound[psi_start]   = psi;
  constraints_lowerbound[v_start]     = v;
  constraints_lowerbound[cte_start]   = cte;
  constraints_lowerbound[epsi_start]  = epsi;

  constraints_upperbound[x_start]     = x;
  constraints_upperbound[y_start]     = y;
  constraints_upperbound[psi_start]   = psi;
  constraints_upperbound[v_start]     = v;
  constraints_upperbound[cte_start]   = cte;
  constraints_upperbound[epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  // return the x and y values of the best solution
  vector<double> result;

  result.push_back(solution.x[delta_start]); // steer angle at t=0
  result.push_back(solution.x[a_start]); // accel at t=0
  // grab all the predicted x and y points of the trajectory
  for (int i = 0; i < N-1; i++){
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
