/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef PID_H
#define PID_H

#include <math/ContinuousAngle.h>
#include <math/vector2d.h>
#include <float.h>

namespace rhoban_ssl
{
namespace robot_control
{
/**
 * @class Pid
 * @brief The Pid class permits to compute a correction
 *
 * This Pid class uses `double` type by default to compute pid.
 * Template specializations of the compute method allow developers to support
 * other types.
 *
 * Types currently supported :
 * double(default), Vector2d, ContinuousAngle
 *
 * @note If a developper tries to use this class with an unsupported type
 * then the code will not compile.
 */
class Pid
{
private:
  /**
   * @brief integral limit
   * By default there is no limitation on the integration
   */
  double integration_limit_ = DBL_MAX;

  double kp_ = 1.0;
  double ki_ = 0.0;
  double kd_ = 0.0;

  double last_error_ = 0.0;
  double sum_errors_ = 0.0;

public:
  Pid() = default;

  /**
   * @brief Pid constructor without integration's limit
   * @param kp : coefficient for the proportional action
   * @param ki : coefficient for the integral action
   * @param kd : coefficient for the derivative action
   */
  Pid(double kp, double ki, double kd);

  /**
   * @brief Pid constructor with integration's limit
   * @param integration_limit : limit on the integration
   * @param kp : coefficient for the proportional action
   * @param ki : coefficient for the integral action
   * @param kd : coefficient for the derivative action
   */
  Pid(double integration_limit, double kp, double ki, double kd);

  /**
   * @brief Permits to set a limit on the integration to
   * limit the action of the integration.
   */
  void setIntegrationLimit(double integration_limit);
  /**
   * @brief set_pid
   * @param kp : coefficient for the proportional action
   * @param ki : coefficient for the integral action
   * @param kd : coefficient for the derivative action
   */
  void setPid(double kp, double ki = 0.0, double kd = 0.0);

  /**
   * @brief Resets the sum of errors use on the integration.
   *
   * It can be use to remove residual errors.
   */
  void resetIntegration();

  /**
   * @brief Resets the last error and the sum of errors use on the integration.
   */
  void reset();

  /**
   * @brief Template declaration for the Pid.
   */
  template <class TYPE>
  TYPE compute(const double& dt, const TYPE& error);
};

///////////////////////////////////////////////////////////////
///               TEMPLATE SPECIALIZATIONS
///////////////////////////////////////////////////////////////

/**
 * @brief Computes a correction from the error according to pid.
 *
 * This Pid class uses `double` by default to compute pid.
 * You may need to add template specialization in this class
 * to support other types.
 *
 * @param dt
 * @param error
 * @return a correction (type : double)
 */
template <>
double Pid::compute<double>(const double& dt, const double& error);

/**
 * @brief Specialization of the Pid for a Vector2d
 * @param dt
 * @param error (type : Vector2d)
 * @return a correction (type : Vector2d)
 */
template <>
Vector2d Pid::compute<Vector2d>(const double& dt, const Vector2d& error);

/**
 * @brief Specialization of the Pid for a ContinuousAngle
 * @param dt
 * @param error
 * @return a ContinuousAngle correction
 */
template <>
ContinuousAngle Pid::compute<ContinuousAngle>(const double& dt, const ContinuousAngle& error);

}  // namespace robot_control
}  // namespace rhoban_ssl

#endif  // PID_H
