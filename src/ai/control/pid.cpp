#include "pid.h"
#include <geometry/Angle.hpp>

#define CALCULUS_ERROR 0.000

PidControl::PidControl():
    velocity_translation(0.0, 0.0), velocity_rotation(0.0)
{ };

PidControl::PidControl(
    const Eigen::Vector2d & velocity_translation,
    ContinuousAngle velocity_rotation
):
    velocity_translation(velocity_translation),
    velocity_rotation(velocity_rotation)
{ };

std::ostream& operator << ( std::ostream & out, const PidControl& control  ){
    out << "[lin vel. : " << control.velocity_translation.transpose()
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
    start_time(0.0), time(0.0), dt(0.0)
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

void PidController::update(double current_time){
    double dt = (current_time - start_time) - this->time;
    if( dt > 0.0 ){
        this->dt = (current_time - start_time) - this->time;
        this->time = (current_time - start_time);
    }
}

double PidController::get_time() const {
    return this->time;
}

Eigen::Vector2d PidController::translation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    ContinuousAngle robot_orientation
) const {
    assert(dt>0);
    if( is_static() ) return Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d xt = goal_position(time);
    Eigen::Vector2d xt_dt = goal_position(time+dt);
    Eigen::Vector2d velocity = (xt_dt - xt )/dt;

    Eigen::Vector2d error = robot_position - xt;

    if( 
        std::abs( error[0] ) < CALCULUS_ERROR and 
        std::abs(error[1] ) < CALCULUS_ERROR
    ){
        error = Eigen::Vector2d(0.0,0.0);
    }

    #if 0
    Eigen::Matrix2d rotation_matrix;
    ContinuousAngle a_r = rotation_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    if( a_r != 0 ){
        rotation_matrix << 
            std::sin(a_r*dt+robot_orientation.value()) - std::sin(robot_orientation.value()), 
            std::cos(a_r*dt+robot_orientation.value()) - std::cos(robot_orientation.value()),
          - std::cos(a_r*dt+robot_orientation.value()) + std::cos(robot_orientation.value()), 
            std::sin(a_r*dt+robot_orientation.value()) - std::sin(robot_orientation.value())
        ;
        rotation_matrix = (a_r*dt)*( rotation_matrix.inverse() );
    }else{
        rotation_matrix << 
            std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
          - std::sin(robot_orientation.value()), std::cos(robot_orientation.value())
        ;
    }
    error /= std::abs( rotation_matrix.determinant() );
    #endif
    
    Eigen::Vector2d absolute_command = (
        velocity - kp_t*error/dt - ki_t*error - kd_t*error/(dt*dt) 
    );


    return  absolute_command; 
}

double PidController::rotation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    ContinuousAngle robot_orientation
) const {
    if( is_static() ) return 0.0;
    ContinuousAngle theta_t = goal_orientation(time);
    ContinuousAngle theta_t_dt = goal_orientation(time+dt);
    //DEBUG( "theta_t : " << theta_t );
    //DEBUG( "theta_t_dt : " << theta_t_dt );
    ContinuousAngle velocity = (theta_t_dt - theta_t )/dt;
    ContinuousAngle error = robot_orientation - theta_t;
    //DEBUG("velocity : " << velocity );
    //DEBUG("theta_t: " << theta_t );
    //DEBUG("robot_orientation: " << robot_orientation );
    //DEBUG("error: " << error );
 
    if( std::abs( error.value() ) <= CALCULUS_ERROR ){
        //DEBUG("ERROR SET TO 0");
        error = 0.0;
    }

    double absolute_command = (
        velocity - error*kp_o/dt - error*ki_o - error*kd_o/(dt*dt) 
    ).value();

    //DEBUG( "absolute command : " << absolute_command );
    return absolute_command;
}

PidControl PidController::absolute_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    ContinuousAngle robot_orientation
) const {
    return PidControl(
        translation_control_in_absolute_frame(
            robot_position, robot_orientation
        ),
        rotation_control_in_absolute_frame(
            robot_position, robot_orientation
        )
    );
}
