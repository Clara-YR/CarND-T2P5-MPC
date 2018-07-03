#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.05;

// variable start index in `vars`
int x_start = 0;  // 0 ~ N-1
int y_start = static_cast<int>(N);  // N ~ 2N-1
int psi_start = static_cast<int>(2 * N);  // 2N ~ 3N-1
int v_start = static_cast<int>(3 * N);  // 3N ~ 4N-1
int cte_start = static_cast<int>(4 * N);  // 4N ~ 5N-1
int epsi_start = static_cast<int>(5 * N);  // 5N ~ 6N-1
int delta_start = static_cast<int>(6 * N);  // 6N ~ 7N-2
int a_start = static_cast<int>(7 * N - 1);  // 7N-1 ~ 8N-2

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

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // initial cost fg[0]
    fg[0] = 0;
    // speed reference
    double ref_v = 35;

    // cost = cte^2 + epsi^2 + (v-ref_v)^2 + delta^2 + a^2 + D_delta^2 + D_a^2
    // The part of the cost based on the reference state.
    for (int t=0; t < N; t++) {
      // cost += cte^2 + epsi^2 + (v - ref_v)^2
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t=0; t < N-1; t++) {
      // cost += delta^2 + a^2
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t=0; t < N-2; t++) {
      // cost += D_delta^2 + D_a^2
      fg[0] += 500 * CppAD::pow((vars[delta_start + t+1] - vars[delta_start + t]), 2);  // TUNE here !
      fg[0] += CppAD::pow((vars[a_start + t+1] - vars[a_start + t]), 2);
    }

    // set the 1st value for each variable
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // set the 2nd ~ Nth values for each variable
    for (int t=1; t < N; t++) {
      // The state at time t+1
      AD<double> x_1 = vars[x_start + t];
      AD<double> y_1 = vars[y_start + t];
      AD<double> psi_1 = vars[psi_start + t];
      AD<double> v_1 = vars[v_start + t];
      AD<double> cte_1 = vars[cte_start + t];
      AD<double> epsi_1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x_0 = vars[x_start + t-1];
      AD<double> y_0 = vars[y_start + t-1];
      AD<double> psi_0 = vars[psi_start + t-1];
      AD<double> v_0 = vars[v_start + t-1];
      AD<double> cte_0 = vars[cte_start + t-1];
      AD<double> epsi_0 = vars[epsi_start + t-1];

      // Only consider the actuation at time t.
      AD<double> delta_0 = vars[delta_start + t-1];
      AD<double> a_0 = vars[a_start + t-1];

      AD<double> f_0 = coeffs[0] + coeffs[1] * x_0;
      AD<double> psi_des_0 = CppAD::atan(coeffs[1]);

      // constraints = state[t+1] - prediction[t+1|t]
      fg[1 + x_start + t] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
      fg[1 + y_start + t] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
      fg[1 + psi_start + t] = psi_1 - (psi_0 - v_0 * delta_0 / Lf * dt);  // delta is positive we rotate counter-clockwise, or turn left
      fg[1 + v_start + t] = v_1 - (v_0 + a_0 * dt);
      fg[1 + cte_start + t] = cte_1 - ((f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt));
      fg[1 + epsi_start + t] = epsi_1 - ((psi_0 - psi_des_0) + v_0 * delta_0 / Lf * dt);
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
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  /* The state [x, y, psi, v, cte, epsi] is a 6 element vector, the actuator [delta, a]
   * is a 2 element vector and there are N timesteps. The number of variables is :
   * 6N + 2(N-1) = 8N - 2;
   */
  size_t n_vars = 8 * N - 2;

  // TODO: Set the number of constraints
  // the number of constraints = fg.size()
  size_t n_constraints = 6 * N + 1;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i=0; i<n_vars; i++) {
    vars[i] = 0;
  }

  // set the initial variable values
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // set the range of values delta to [-25, 25] in radians
  for (i=delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;  // -25/180 * PI
    vars_upperbound[i] =  0.436332;  //  25/180 * PI
  }
  // set the range of values a to [-1, 1]
  for (i=a_start; i<n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i=0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
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
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
    
  return {solution.x[delta_start], solution.x[a_start]};
}

