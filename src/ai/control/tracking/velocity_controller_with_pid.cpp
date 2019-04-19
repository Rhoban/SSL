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
#include "velocity_controller_with_pid.h"

#include <assert.h>
#include <math.h>
#include <math/matrix2d.h>
#include <debug.h>

// TODO : load values from config files
#define LIMITE_LINEAR_ACCELERATION 200.0
#define LIMITE_ANGULAR_ACCELERATION 3.0

#define USE_PID true

namespace rhoban_ssl
{
namespace robot_control
{
namespace tracking
{
Control VelocityControllerWithPid::getLimitedControl() const
{
  bool absolute = true;
  Control ctrl(absolute);

  ctrl.linear_velocity = computed_limited_linear_velocity_;
  ctrl.angular_velocity = computed_limited_angular_velocity_;

  // TODO limitation :
  //  if linear velocity is limited -> decrease the angular velocity
  //  if angulara velocity is limited -> decrease the linear velocity ( TODO formula )

  // DEBUG("consigne L: " << ctrl.linear_velocity);
  // DEBUG("consigne W: " << ctrl.angular_velocity);

  return ctrl;
}

void VelocityControllerWithPid::computeLimitedControl()
{
  Vector2d no_limited_linear_velocity = computeNoLimitedLinearVelocity(current_linear_position_);
  ContinuousAngle no_limited_angular_velocity = computeNoLimitedAngularVelocity(current_angular_position_);

  // TODO faire la limitation ici
  computed_limited_linear_velocity_ = no_limited_linear_velocity;
  computed_limited_angular_velocity_ = no_limited_angular_velocity;

  //  computed_limited_angular_velocity_ = 0;
}

void VelocityControllerWithPid::setLinearVelocityPid(double kp, double ki, double kd)
{
  linear_velocity_pid_.setPid(kp, ki, kd);
}

void VelocityControllerWithPid::setAngularVelocityPid(double kp, double ki, double kd)
{
  angular_velocity_pid_.setPid(kp, ki, kd);
}

Vector2d VelocityControllerWithPid::computeNoLimitedLinearVelocity(const Vector2d& robot_position)
{
  assert(dt_ > 0);

  if (isStatic())
  {
    return Vector2d(0.0, 0.0);
  }

  Vector2d no_limited_linear_velocity = linearVelocity(time_);
  //DEBUG("Error L: " << no_limited_linear_velocity );

#if false
  Vector2d error = linearPosition(time_) - robot_position;

//  if( error.getX() <= 0.001 && error.getY() <= 0.001 ) {
//    linear_velocity_pid_.resetIntegration();
//    DEBUG("reset PID integration -> CALIBRATE");
//  } else {
    linear_velocity_pid_.setPid(1.5,0,0);
    Vector2d correction = linear_velocity_pid_.compute(dt_, error);
    DEBUG("error " << error);
    DEBUG("correction " << correction );
    no_limited_linear_velocity += correction;
//  }

#endif
  return no_limited_linear_velocity;
}

ContinuousAngle
VelocityControllerWithPid::computeNoLimitedAngularVelocity(const ContinuousAngle& robot_angular_position)
{
  assert(dt_ > 0);
  if (isStatic())
  {
    return ContinuousAngle(0);
  }

  ContinuousAngle no_limited_angular_velocity = angularVelocity(time_);

#if false
  ContinuousAngle error = angularPosition(time_) - robot_angular_position;

 // DEBUG("theta error : " << error.value());

  no_limited_angular_velocity += angular_velocity_pid_.compute(dt_, error);
#endif

  return no_limited_angular_velocity;
}

}  // namespace tracking
}  // namespace robot_control
}  // namespace rhoban_ssl
