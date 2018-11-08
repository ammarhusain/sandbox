// A simple example of using the Ceres minimizer.
//
// Minimize 0.5 (10 - x)^2 using jacobian matrix computed using
// automatic differentiation.
#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// Automatic Differentiation.
// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

// Numeric Differentiation.
// A cost functor that implements the residual r = 10 - x.
struct NumericCostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

// Analytical Differentiation.
// A CostFunction implementing analytically derivatives for the
// function f(x) = 10 - x.
class QuadraticCostFunction
  : public SizedCostFunction<1 /* number of residuals */,
                             1 /* size of first parameter */> {
 public:
  virtual ~QuadraticCostFunction() {}
  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    double x = parameters[0][0];
    // f(x) = 10 - x.
    residuals[0] = 10 - x;
    // f'(x) = -1. Since there's only 1 parameter and that parameter
    // has 1 dimension, there is only 1 element to fill in the
    // jacobians.
    //
    // Since the Evaluate function can be called with the jacobians
    // pointer equal to NULL, the Evaluate function must check to see
    // if jacobians need to be computed.
    //
    // For this simple problem it is overkill to check if jacobians[0]
    // is NULL, but in general when writing more complex
    // CostFunctions, it is possible that Ceres may only demand the
    // derivatives w.r.t. a subset of the parameter blocks.
    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;
  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  /*
  // Set up the only cost function (also known as residual). This uses
  // numeric differentiation to obtain the derivative (jacobian).
  CostFunction* num_cost_function =
      new NumericDiffCostFunction<NumericCostFunctor, CENTRAL, 1, 1> (new NumericCostFunctor);
  problem.AddResidualBlock(num_cost_function, NULL, &x);

    // Set up the only cost function (also known as residual).
  CostFunction* analytic_cost_function = new QuadraticCostFunction;
  problem.AddResidualBlock(analytic_cost_function, NULL, &x);
  */

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
