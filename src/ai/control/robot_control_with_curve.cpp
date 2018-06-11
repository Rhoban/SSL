/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "robot_control_with_curve.h"

void RobotControlWithCurve::set_movement(
    const std::function<Vector2d (double u)> & translation,
    double translation_velocity, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double angular_acceleration, 
    double calculus_step, double current_time, double current_dt
){
    curve = CurveForRobot(
        translation,
        translation_velocity,
        translation_acceleration,
        rotation,
        angular_velocity, 
        angular_acceleration, 
        calculus_step
    );
    init_time(current_time, current_dt);
    set_static(false);
}

ContinuousAngle RobotControlWithCurve::goal_orientation( double t ) const {
    return curve.rotation(t);
}

Vector2d RobotControlWithCurve::goal_position( double t ) const {
    return curve.translation(t);
}

RobotControlWithCurve::RobotControlWithCurve():
    curve(
        [](double t){ return Vector2d(1.0,0.0); },
        0.0, 1.0, 
        [](double t){ return 1.0; },
        0.0, 1.0, 
        0.001
    )
{ }
