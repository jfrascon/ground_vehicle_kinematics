#include "ground_vehicle_kinematics/solvers/svd_least_squares_solver.hpp"

#include "ground_vehicle_kinematics/helpers.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/SVD>

namespace ground_vehicle_kinematics
{
  //////////////////////////////////////////////////////////////////////////////
  // SvdLeastSquaresSolverConfig
  //////////////////////////////////////////////////////////////////////////////

  SvdLeastSquaresSolverConfig::SvdLeastSquaresSolverConfig(const double relative_singular_value_threshold,
                                                           const double tikhonov_lambda):
    relative_singular_value_threshold_{relative_singular_value_threshold},
    tikhonov_lambda_{tikhonov_lambda}
  {
    validate();
  }

  void SvdLeastSquaresSolverConfig::validate() const
  {
    detail::ensure_non_negative_finite(relative_singular_value_threshold_, "relative_singular_value_threshold");
    detail::ensure_non_negative_finite(tikhonov_lambda_, "tikhonov_lambda");
  }

  //////////////////////////////////////////////////////////////////////////////
  // SvdLeastSquaresSolver
  //////////////////////////////////////////////////////////////////////////////

  SvdLeastSquaresSolver::SvdLeastSquaresSolver(const Eigen::MatrixXd& A, const SvdLeastSquaresSolverConfig& cfg):
    diagnostics_{std::numeric_limits<double>::quiet_NaN(),
                 std::numeric_limits<double>::quiet_NaN(),
                 0U,
                 std::numeric_limits<double>::quiet_NaN()},
    cfg_{cfg}
  {
    reset(A);
  }

  Eigen::VectorXd SvdLeastSquaresSolver::compute_residual(Eigen::Ref<const Eigen::VectorXd> x,
                                                          Eigen::Ref<const Eigen::VectorXd> b) const
  {
    if(x.rows() != A_.cols())
    {
      throw std::invalid_argument("Solution vector x has invalid size for the current matrix A.");
    }

    if(b.rows() != A_.rows())
    {
      throw std::invalid_argument("Right-hand side vector b has invalid size for the current matrix A.");
    }

    return (A_ * x) - b;
  }

  void SvdLeastSquaresSolver::reset(const Eigen::MatrixXd& A)
  {
    // The solver needs a real matrix to decompose.
    if(A.rows() == 0 || A.cols() == 0)
    {
      throw std::invalid_argument("Least-squares matrix A must be non-empty.");
    }

    // Store the new least-squares matrix.
    A_ = A;

    // Build a stable pseudoinverse for the new matrix A.
    // Very weak singular directions are discarded.
    // The kept directions are damped with Tikhonov regularization.

    // Decompose A into U, S and V.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Read the singular values from the decomposition.
    const auto& singular_values{svd.singularValues()};
    // The largest singular value sets the scale for the cutoff test.
    const double max_singular_value{singular_values.size() > 0 ? singular_values.maxCoeff() : 0.0};

    // Build the diagonal part of the regularized pseudo-inverse.
    Eigen::MatrixXd regularized_inverse{Eigen::MatrixXd::Zero(singular_values.size(), singular_values.size())};

    // Track the smallest singular value that survives the cutoff.
    double min_retained_singular_value{std::numeric_limits<double>::infinity()};
    // Track how many singular values we keep.
    std::size_t retained_rank{0};

    for(Eigen::Index i{0}; i < singular_values.size(); ++i)
    {
      const double singular_value{singular_values[i]};
      // Drop singular values that are too small relative to the largest one.
      const bool keep{singular_value > (cfg_.relative_singular_value_threshold() * max_singular_value)};

      if(keep)
      {
        // Invert the singular value with Tikhonov regularization.
        regularized_inverse(i, i) = singular_value / ((singular_value * singular_value) + cfg_.tikhonov_lambda());
        // Keep the smallest retained singular value for diagnostics.
        min_retained_singular_value = std::min(min_retained_singular_value, singular_value);
        ++retained_rank;
      }
    }

    // Rebuild the regularized pseudo-inverse from V, S^+ and U^T.
    Aplus_regularized_ = svd.matrixV() * regularized_inverse * svd.matrixU().transpose();

    // Store diagnostics for inspection and debugging.
    diagnostics_.set_max_singular_value(max_singular_value);
    diagnostics_.set_min_retained_singular_value((retained_rank > 0) ? min_retained_singular_value : 0.0);
    diagnostics_.set_retained_rank(retained_rank);
    diagnostics_.set_retained_condition_number((retained_rank > 0) ?
                                                 (max_singular_value / min_retained_singular_value) :
                                                 std::numeric_limits<double>::infinity());
  }

  Eigen::VectorXd SvdLeastSquaresSolver::solve(Eigen::Ref<const Eigen::VectorXd> b) const
  {
    if(b.rows() != A_.rows())
    {
      throw std::invalid_argument("Right-hand side vector b has invalid size for the current matrix A.");
    }

    return Aplus_regularized_ * b;
  }
}  // namespace ground_vehicle_kinematics
