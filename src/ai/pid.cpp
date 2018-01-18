#include "pid.h"
#include <geometry/Angle.hpp>

#define CALCULUS_ERROR 0.000

Control::Control():
    velocity_translation(0.0, 0.0),
    velocity_rotation(0.0),
    kick(false)
{ };



PidControl::PidControl():
    PidControl(1.0, 0.0, 0.0)
{ }
PidControl::PidControl( double p, double i=0.0, double d=0.0 ):
    PidControl(p, i, d, p, i, d)
{ }
PidControl::PidControl(
    double p_t, double i_t, double d_t, 
    double p_o, double i_o, double d_o 
):
    kp_t(p_t), ki_t(i_t), kd_t(d_t),
    kp_o(p_o), ki_o(i_o), kd_o(d_o),
    static_robot(true),
    start_time(0.0), time(0.0), dt(0.0)
{ }


void PidControl::set_static(bool value = true){
    static_robot = value;
}
bool PidControl::is_static() const {
    return static_robot;
}

void PidControl::init_time(double start_time){
    this->start_time = start_time;
    this->dt = 0.0;
    this->time = 0.0;
}

void PidControl::set_orientation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_o = kp;
    this->ki_o = ki;
    this->kd_o = kd;
}

void PidControl::set_translation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_t = kp;
    this->ki_t = ki;
    this->kd_t = kd;
}

void PidControl::set_pid( double kp, double ki=0.0, double kd=0.0 ){
    set_orientation_pid( kp, ki, kd );
    set_translation_pid( kp, ki, kd );
}

void PidControl::update(double current_time){
    this->dt = (current_time - start_time) - this->time;
    this->time = (current_time - start_time);
}

Eigen::Vector2d PidControl::translation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
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

    Eigen::Vector2d absolute_command = (
        velocity - kp_t*error/dt - ki_t*error - kd_t*error/(dt*dt) 
    );

    if( absolute_command.norm() >= TRANSLATION_VELOCITY_LIMIT ){
        absolute_command *= (  TRANSLATION_VELOCITY_LIMIT / (2*absolute_command.norm()) );
        std::cerr << "Translation velocity limit !" << std::endl;
    }

    return  absolute_command; 
}

double PidControl::rotation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {
    if( is_static() ) return 0.0;
    double theta_t = goal_orientation(time);
    double theta_t_dt = goal_orientation(time+dt);
    double velocity = (theta_t_dt - theta_t )/dt;
    Angle a( rad2deg(robot_orientation - theta_t) );
    double error = deg2rad( a.getSignedValue() );
    if( error >0.1 ){
        std::cout << "ERROR " << error << std::endl;
    }
 
    if( std::abs( error ) <= CALCULUS_ERROR ){
        error = 0.0;
    }
    double absolute_command = (
        velocity - kp_o*error/dt - ki_o*error - kd_o*error/(dt*dt) 
    );

    if( std::abs( absolute_command ) >= ROTATION_VELOCITY_LIMIT ){
        absolute_command *= (
            ROTATION_VELOCITY_LIMIT / std::abs(absolute_command) 
        );
        std::cerr << "Rotation velocity limit !" << std::endl;
    }

    return absolute_command;
}

Control PidControl::absolute_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {
    Control res;

    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 
          std::cos(robot_orientation), std::sin(robot_orientation),
        - std::sin(robot_orientation), std::cos(robot_orientation)
    ;

    Eigen::Vector2d absolute_translation = translation_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    double absolute_rotation = rotation_control_in_absolute_frame(
        robot_position, robot_orientation
    );

    res.velocity_translation = rotation_matrix * absolute_translation;
    res.velocity_rotation = absolute_rotation;

    return res;
}

Control PidControl::relative_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {

    Control res;
    Eigen::Matrix2d rotation_matrix;

    Eigen::Vector2d a_t = translation_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    double a_r = rotation_control_in_absolute_frame(
        robot_position, robot_orientation
    );

    if( a_r != 0 ){
        rotation_matrix << 
            std::sin(a_r*dt+robot_orientation) - std::sin(robot_orientation), 
            std::cos(a_r*dt+robot_orientation) - std::cos(robot_orientation),
          - std::cos(a_r*dt+robot_orientation) + std::cos(robot_orientation), 
            std::sin(a_r*dt+robot_orientation) - std::sin(robot_orientation)
        ;
        rotation_matrix = (a_r*dt)*( rotation_matrix.inverse() );
    }else{
        rotation_matrix << 
            std::cos(robot_orientation), std::sin(robot_orientation),
          - std::sin(robot_orientation), std::cos(robot_orientation)
        ;
    }
    res.velocity_translation = rotation_matrix * a_t;
    res.velocity_rotation = a_r;
    return res;
}
