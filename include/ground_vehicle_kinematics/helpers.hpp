#pragma once

/**
 * @file helpers.hpp
 * @brief Small validation helpers shared by the package.
 */

#include <cmath>
#include <stdexcept>
#include <string>

namespace ground_vehicle_kinematics
{
  namespace detail
  {
    /**
     * @brief Validate that a string field is not empty.
     * @param value Value to validate.
     * @param field_name Field identifier used in error messages.
     * @throws std::invalid_argument If @p value is empty.
     */
    inline void ensure_non_empty(const std::string& value, const std::string& field_name)
    {
      if(value.empty())
      {
        throw std::invalid_argument(std::string(field_name) + " cannot be empty.");
      }
    }

    /**
     * @brief Validate that a numeric field is finite.
     * @param value Value to validate.
     * @param field_name Field identifier used in error messages.
     * @throws std::invalid_argument If @p value is NaN or infinite.
     */
    inline void ensure_finite(const double value, const std::string& field_name)
    {
      if(!std::isfinite(value))
      {
        throw std::invalid_argument(std::string(field_name) + " must be finite.");
      }
    }

    /**
     * @brief Validate that a numeric field is finite and non-negative (>= 0).
     * @param value Value to validate.
     * @param field_name Field identifier used in error messages.
     * @throws std::invalid_argument If @p value is negative, NaN or infinite.
     */
    inline void ensure_non_negative_finite(const double value, const std::string& field_name)
    {
      ensure_finite(value, field_name);

      if(value < 0.0)
      {
        throw std::invalid_argument(std::string(field_name) + " must be a finite value >= 0.");
      }
    }

    /**
     * @brief Validate that a numeric field is finite and strictly positive (> 0).
     * @param value Value to validate.
     * @param field_name Field identifier used in error messages.
     * @throws std::invalid_argument If @p value is non-positive, NaN or infinite.
     */
    inline void ensure_positive_finite(const double value, const std::string& field_name)
    {
      ensure_finite(value, field_name);

      // Avoid "<=" so the floating-point comparison does not include an exact equality branch.
      if(!(value > 0.0))
      {
        throw std::invalid_argument(std::string(field_name) + " must be a finite positive value.");
      }
    }
  }  // namespace detail
}  // namespace ground_vehicle_kinematics
