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

#include "pid.h"
#include <rhoban_utils/angle.h>
#include <math/matrix2d.h>

#define CALCULUS_ERROR 0.000
#define LIMITE 200.0
#define LIMITE_ROT 3.0

PidControl PidControl::relative_control(
    const ContinuousAngle & robot_orientation,
    double dt
) const {
    Matrix2d rotation_matrix;

    const Vector2d & a_t = velocity_translation;
    const ContinuousAngle & a_r =  velocity_rotation;

    if( std::fabs(a_r.value()) > CALCULUS_ERROR ){
        rotation_matrix = Matrix2d(
            std::sin((a_r*dt+robot_orientation).value()) - std::sin(robot_orientation.value()),
            std::cos((a_r*dt+robot_orientation).value()) - std::cos(robot_orientation.value()),
          - std::cos((a_r*dt+robot_orientation).value()) + std::cos(robot_orientation.value()),
            std::sin((a_r*dt+robot_orientation).value()) - std::sin(robot_orientation.value())
        );
        rotation_matrix = (a_r*dt).value()*( rotation_matrix.inverse() );
    }else{
        rotation_matrix = Matrix2d(
            std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
          - std::sin(robot_orientation.value()), std::cos(robot_orientation.value())
        );
    }
    return PidControl( rotation_matrix * a_t, a_r );
}

PidControl::PidControl():
    velocity_translation(0.0, 0.0), velocity_rotation(0.0)
{ };

PidControl::PidControl(
    const Vector2d & velocity_translation,
    ContinuousAngle velocity_rotation
):
    velocity_translation(velocity_translation),
    velocity_rotation(velocity_rotation)
{ };

std::ostream& operator << ( std::ostream & out, const PidControl& control  ){
    out << "[lin vel. : " << control.velocity_translation
        << ", ang vel. : " << control.velocity_rotation << "]";
    return out;
}

PidController::PidController():
    PidController(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
{ }
PidController::PidController(
    double p_t, double i_t, double d_t,
    double p_o, double i_o, double d_o
):
    kp_t(p_t), ki_t(i_t), kd_t(d_t),
    kp_o(p_o), ki_o(i_o), kd_o(d_o),
    static_robot(true),
    start_time(0.0), time(0.0), dt(0.0),
    acc_r(0.0), acc(0.0,0.0),
    ancient_pos(0.0,0.0), ancient_orientation(0.0)
{ }


void PidController::set_static(bool value = true){
    static_robot = value;
}
bool PidController::is_static() const {
    return static_robot;
}

double PidController::get_dt() const {
    return dt;
}

void PidController::init_time(double start_time, double dt ){
    assert( dt > 0.0 );
    this->start_time = start_time;
    this->dt = dt;
    this->time = 0.0;
}

void PidController::set_orientation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_o = kp;
    this->ki_o = ki;
    this->kd_o = kd;
}

void PidController::set_translation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_t = kp;
    this->ki_t = ki;
    this->kd_t = kd;
}

void PidController::update(
    double current_time,
    const Vector2d & robot_position,
    const ContinuousAngle & robot_orientation
){
    double dt = (current_time - start_time) - this->time;
    if( dt > 0.0 ){
        this->dt = (current_time - start_time) - this->time;
        this->time = (current_time - start_time);
    }

    compute_no_limited_translation_control( robot_position );
    compute_no_limited_angular_control(robot_orientation);
}

double PidController::get_time() const {
    return this->time;
}

Vector2d PidController::no_limited_translation_control() const {
    return no_limited_translation_control_value;
}

void PidController::compute_no_limited_translation_control(
    const Vector2d & robot_position
){
    assert(dt>0);
    if( is_static() ){
        no_limited_translation_control_value = Vector2d(0.0, 0.0);
        return;
    }
    Vector2d xt = goal_position(time);
    Vector2d xt_dt = goal_position(time+dt);
    Vector2d velocity = (robot_position - ancient_pos )/dt;


    Vector2d error =  xt - robot_position;

    if(
        std::fabs( error[0] ) < CALCULUS_ERROR and
        std::fabs( error[1] ) < CALCULUS_ERROR
    ){
        error = Vector2d(0.0,0.0);
    }
    //DEBUG("error: " << error );
    //std::cout<<time<<" "<<error[0]<<" "<<error[1]<<std::endl;


    acc_dt[0] = acc[0] - ki_t*error[0]*dt;
    if (acc_dt[0] > LIMITE){
      acc[0] = LIMITE;
    }
    else if(acc_dt[0] < -LIMITE){
      acc[0] = -LIMITE;
    }
    else{
      acc[0] = acc_dt[0];
    }

    acc_dt[1] = acc[1] - ki_t*error[1]*dt;
    if (acc_dt[1] > LIMITE){
      acc[1] = LIMITE;
    }
    else if(acc_dt[1] < -LIMITE){
      acc[1] = -LIMITE;
    }
    else{
      acc[1] = acc_dt[1];
    }

    no_limited_translation_control_value = (
        kp_t*error + acc + kd_t*velocity
    );

    //DEBUG( "PID" << time<<" "<<no_limited_translation_control_value[0]<<" "<<no_limited_translation_control_value[1]);

    ancient_pos = robot_position;

}

void PidController::compute_no_limited_angular_control(
    const ContinuousAngle & robot_orientation
){
    if( is_static() ){
        no_limited_angular_control_value = 0.0;
        return;
    };
    ContinuousAngle theta_t = goal_orientation(time);
    ContinuousAngle theta_t_dt = goal_orientation(time+dt);
    //DEBUG( "theta_t : " << theta_t );
    //DEBUG( "theta_t_dt : " << theta_t_dt );
    ContinuousAngle velocity = (robot_orientation - ancient_orientation )/dt;
    ContinuousAngle error = theta_t - robot_orientation;
    //DEBUG("velocity : " << velocity );
    //DEBUG("theta_t: " << theta_t );
    //DEBUG("robot_orientation: " << robot_orientation );
    //DEBUG("error: " << error );

    if( std::fabs( error.value() ) <= CALCULUS_ERROR ){
        //DEBUG("ERROR SET TO 0");
        error = 0.0;
    }
    acc_r_dt = acc_r - ki_o*error.value()*dt;
    if (acc_r_dt > LIMITE_ROT){
      acc_r = LIMITE_ROT;
    }
    else if(acc_r_dt < -LIMITE_ROT){
      acc_r = -LIMITE_ROT;
    }
    else{
      acc_r = acc_r_dt;
    }

    no_limited_angular_control_value = (
        kp_o*error.value() + acc_r + kd_o*velocity.value()
    );
    ancient_orientation = robot_orientation;

    //DEBUG( "kpt : " << kp_o );
}

double PidController::no_limited_angular_control() const {
    return no_limited_angular_control_value;
}

PidControl PidController::no_limited_control() const {
    return PidControl(
        no_limited_translation_control(),
        no_limited_angular_control()
    );
}
