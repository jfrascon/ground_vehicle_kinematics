#pragma once

/**
 * @file svd_least_squares_solver.hpp
 * @brief Internal least-squares solver based on SVD.
 */

#include <cstddef>

#include <Eigen/Dense>

namespace ground_vehicle_kinematics
{
  //////////////////////////////////////////////////////////////////////////////
  // SvdLeastSquaresSolverConfig
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Numerical parameters of the SVD least-squares solver.
   */
  class SvdLeastSquaresSolverConfig
  {
    public:
      /**
       * @brief Build one SVD solver config.
       * @param relative_singular_value_threshold Relative threshold used to discard weak singular
       * directions.
       * @param tikhonov_lambda Tikhonov damping factor.
       */
      explicit SvdLeastSquaresSolverConfig(double relative_singular_value_threshold = 1e-6,
                                           double tikhonov_lambda = 1e-4);

      /** @return Relative threshold used to discard weak singular directions. */
      double relative_singular_value_threshold() const
      {
        return relative_singular_value_threshold_;
      }

      /** @return Tikhonov damping factor. */
      double tikhonov_lambda() const
      {
        return tikhonov_lambda_;
      }

    private:
      double relative_singular_value_threshold_;
      double tikhonov_lambda_;

      /**
       * @brief Validate the current SVD solver config.
       * @throws std::invalid_argument If any stored field is invalid.
       */
      void validate() const;
  };

  //////////////////////////////////////////////////////////////////////////////
  // SvdLeastSquaresDiagnostics
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Diagnostics of the current SVD factorization.
   */
  class SvdLeastSquaresDiagnostics
  {
    public:
      /**
       * @brief Build one diagnostics snapshot.
       * @param max_singular_value Largest singular value of the current matrix A.
       * @param min_retained_singular_value Smallest singular value kept after filtering.
       * @param retained_rank Effective rank after truncation.
       * @param retained_condition_number Condition number of the retained singular values.
       */
      SvdLeastSquaresDiagnostics(double max_singular_value,
                                 double min_retained_singular_value,
                                 std::size_t retained_rank,
                                 double retained_condition_number):
        max_singular_value_{max_singular_value},
        min_retained_singular_value_{min_retained_singular_value},
        retained_rank_{retained_rank},
        retained_condition_number_{retained_condition_number}
      {}

      /** @return Largest singular value of the current matrix A. */
      double max_singular_value() const
      {
        return max_singular_value_;
      }

      /** @return Smallest singular value kept after filtering. */
      double min_retained_singular_value() const
      {
        return min_retained_singular_value_;
      }

      /** @return Condition number of the retained singular values. */
      double retained_condition_number() const
      {
        return retained_condition_number_;
      }

      /** @brief Update the largest singular value of the current matrix A. */
      void set_max_singular_value(const double max_singular_value)
      {
        max_singular_value_ = max_singular_value;
      }

      /** @brief Update the smallest singular value kept after filtering. */
      void set_min_retained_singular_value(const double min_retained_singular_value)
      {
        min_retained_singular_value_ = min_retained_singular_value;
      }

      /** @brief Update the condition number of the retained singular values. */
      void set_retained_condition_number(const double retained_condition_number)
      {
        retained_condition_number_ = retained_condition_number;
      }

      /** @brief Update the effective rank after truncation. */
      void set_retained_rank(const std::size_t retained_rank)
      {
        retained_rank_ = retained_rank;
      }

      /** @return Effective rank after truncation. */
      std::size_t retained_rank() const
      {
        return retained_rank_;
      }

    private:
      double max_singular_value_;
      double min_retained_singular_value_;
      std::size_t retained_rank_;
      double retained_condition_number_;
  };

  //////////////////////////////////////////////////////////////////////////////
  // SvdLeastSquaresSolver
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Solve least-squares problems with a precomputed SVD-based pseudoinverse.
   *
   * The factorization is built in the constructor or in @ref reset.
   * Calls to @ref solve reuse that precomputed result.
   */
  class SvdLeastSquaresSolver
  {
    public:
      /**
       * @brief Build the solver for a given matrix A.
       * @param A Least-squares matrix.
       * @param cfg Numerical parameters controlling truncation and damping.
       * @throws std::invalid_argument If the matrix or solver parameters are invalid.
       */
      explicit SvdLeastSquaresSolver(const Eigen::MatrixXd& A,
                                     const SvdLeastSquaresSolverConfig& cfg = SvdLeastSquaresSolverConfig{});

      /**
       * @brief Compute the residual vector r = A x - b.
       * @param x Candidate solution vector.
       * @param b Right-hand side vector.
       * @return Residual vector.
       * @throws std::invalid_argument If the input dimensions do not match the current matrix
       * dimensions.
       */
      Eigen::VectorXd compute_residual(Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> b) const;

      /**
       * @brief Get diagnostics of the current factorization.
       * @return Const reference to the current diagnostics.
       */
      const SvdLeastSquaresDiagnostics& diagnostics() const noexcept
      {
        return diagnostics_;
      }

      /**
       * @brief Recompute the factorization for a new matrix A.
       * @param A Least-squares matrix.
       * @throws std::invalid_argument If the matrix dimensions are invalid.
       */
      void reset(const Eigen::MatrixXd& A);

      /**
       * @brief Solve x = A^+ b using the precomputed regularized pseudoinverse.
       * @param b Right-hand side vector.
       * @return Least-squares solution vector.
       * @throws std::invalid_argument If @p b does not match the current matrix dimensions.
       */
      Eigen::VectorXd solve(Eigen::Ref<const Eigen::VectorXd> b) const;

    private:
      /**
       * @brief Matrix for the current least-squares problem.
       */
      Eigen::MatrixXd A_;

      /**
       * @brief Precomputed regularized pseudoinverse of A.
       */
      Eigen::MatrixXd Aplus_regularized_;

      /**
       * @brief Diagnostics of the current factorization.
       */
      SvdLeastSquaresDiagnostics diagnostics_;

      /**
       * @brief Numerical config of the SVD least-squares solver.
       */
      SvdLeastSquaresSolverConfig cfg_;
  };

}  // namespace ground_vehicle_kinematics
