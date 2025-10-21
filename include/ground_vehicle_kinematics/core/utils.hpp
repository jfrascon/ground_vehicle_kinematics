#pragma once

#include <cmath>

namespace ground_vehicle_kinematics
{
  inline bool nearly_equal(double a, double b, double tol = 1e-9) noexcept
  {
    return std::abs(a - b) <= tol;
  }
}  // namespace ground_vehicle_kinematics
