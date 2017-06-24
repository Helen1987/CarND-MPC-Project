#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 6;
double dt = 0.4;

size_t const x_start = 0;
size_t const y_start = x_start + N;
size_t const psi_start = y_start + N;
size_t const v_start = psi_start + N;
size_t const cte_start = v_start + N;
size_t const epsi_start = cte_start + N;
size_t const delta_start = epsi_start + N;
size_t const a_start = delta_start + N - 1;

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

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_v = 17;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  vector<double> tuning_coeff;

  FG_eval(Eigen::VectorXd coeffs, vector<double> tuning_coeff) {
    this->coeffs = coeffs;
    this->tuning_coeff = tuning_coeff;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += tuning_coeff[0] * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += tuning_coeff[1] * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += tuning_coeff[2] * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    //std::cout << "cte: " << vars[cte_start] << std::endl;
    //std::cout << "epsi: " << vars[epsi_start] << std::endl;
    //std::cout << "v: " << vars[v_start] << std::endl;

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += tuning_coeff[3] * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += tuning_coeff[4] * CppAD::pow(vars[a_start + t], 2);
    }
    //std::cout << "delta: " << vars[delta_start] << std::endl;
    //std::cout << "a: " << vars[a_start] << std::endl;

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += tuning_coeff[5] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += tuning_coeff[6] * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    //std::cout << "d_delta: " << vars[delta_start + 1] - vars[delta_start] << std::endl;
    //std::cout << "a_delta: " << vars[a_start + 1] - vars[a_start] << std::endl;

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (size_t t = 1; t < N; t++) {
      // time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // time t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * (x0 * x0) + coeffs[3] * (x0 * x0 * x0);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
      // x(t+1) = x(t) + v(t) * cos(psi(t)) * dt
      // y(t+1) = y(t) + v(t) * sin(psi(t)) * dt
      // psi(t+1) = psi(t) + v(t) / Lf * delta(t) * dt
      // v(t+1) = v(t) + a(t) * dt
      // cte(​t + 1)​ = f(x(​t)​​)−y(​t)​​ + (v(​t)*sin(eψ(​t)​​)*dt)
      // eψ(​t + 1) = ψ(​t)−ψdes(​t) + (​v(​t)*delta(t)*dt/Lf)
      // constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0)*dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0*delta0*dt / Lf);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0*CppAD::sin(epsi0)*dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + v0*delta0*dt/Lf);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::setParameters(vector<double> parameters) {
  tuning_coeff = parameters;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = 6 * N + 2 * (N-1);
  size_t n_constraints = N*6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  int const per_act = N-1;
  for (i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (int i = 0; i < per_act; ++i) {
    // delta
    vars_lowerbound[delta_start + i] = -0.436332;
    vars_upperbound[delta_start + i] = 0.436332;
    // a
    vars_lowerbound[a_start + i] = -1;
    vars_upperbound[a_start + i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, tuning_coeff);

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
  vector<double> result { solution.x[delta_start], solution.x[a_start] };
  for (i = x_start; i < y_start; ++i) {
    result.push_back(solution.x[i]);
  }
  for (i = y_start; i < psi_start; ++i) {
    result.push_back(solution.x[i]);
  }
  return result;
}
