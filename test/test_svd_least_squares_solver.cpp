#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "ground_vehicle_kinematics/solvers/svd_least_squares_solver.hpp"

namespace ground_vehicle_kinematics
{
  TEST(SvdLeastSquaresSolverTest, SolvesOverdeterminedSystemAndComputesResidual)
  {
    Eigen::Matrix<double, 3, 2> A;
    A << 1.0, 0.0, 0.0, 1.0, 1.0, 1.0;

    const Eigen::Vector2d expected_x{2.0, -1.0};
    const Eigen::Vector3d b{A * expected_x};

    const SvdLeastSquaresSolver solver{A, SvdLeastSquaresSolverConfig{0.0, 0.0}};
    const Eigen::VectorXd x{solver.solve(b)};
    const Eigen::VectorXd residual{solver.compute_residual(x, b)};

    EXPECT_NEAR(x(0), expected_x(0), 1e-12);
    EXPECT_NEAR(x(1), expected_x(1), 1e-12);
    EXPECT_NEAR(residual.norm(), 0.0, 1e-12);
  }

  TEST(SvdLeastSquaresSolverTest, ExposesDiagnosticsAndReset)
  {
    Eigen::Matrix2d A;
    A << 3.0, 0.0, 0.0, 1.0;

    SvdLeastSquaresSolver solver{A, SvdLeastSquaresSolverConfig{0.0, 0.0}};
    const auto& diagnostics{solver.diagnostics()};

    EXPECT_NEAR(diagnostics.max_singular_value(), 3.0, 1e-12);
    EXPECT_NEAR(diagnostics.min_retained_singular_value(), 1.0, 1e-12);
    EXPECT_EQ(diagnostics.retained_rank(), 2U);
    EXPECT_NEAR(diagnostics.retained_condition_number(), 3.0, 1e-12);

    Eigen::Matrix2d B;
    B << 1.0, 0.0, 0.0, 2.0;

    solver.reset(B);
    const Eigen::Vector2d b{1.0, 4.0};
    const Eigen::VectorXd x{solver.solve(b)};

    EXPECT_NEAR(x(0), 1.0, 1e-12);
    EXPECT_NEAR(x(1), 2.0, 1e-12);
    EXPECT_NEAR(solver.diagnostics().retained_condition_number(), 2.0, 1e-12);
  }

}  // namespace ground_vehicle_kinematics
