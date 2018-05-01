#include "shooter.h"

using namespace rhoban_utils;

namespace RhobanSSL {
namespace Robot_behavior {

Shooter::Shooter(
    const Vector2d & goal_center, double robot_radius,
    double front_size, double radius_ball,
    double translation_velocity,
    double translation_acceleration,
    double angular_velocity,
    double angular_acceleration,
    double calculus_step,
    double time, double dt
){
    this->goal_center = goal_center;
    this->robot_radius = robot_radius;
    this->translation_velocity = translation_velocity;
    this->translation_acceleration = translation_acceleration;
    this->angular_velocity = angular_velocity;
    this->angular_acceleration = angular_acceleration;
    this->calculus_step = calculus_step;
    this->front_size = front_size;
    this->radius_ball = radius_ball;
    robot_control.init_time( time, dt );
} 

void Shooter::set_translation_pid( double kp, double ki, double kd ){
    robot_control.set_translation_pid( kp, ki, kd );
}

void Shooter::set_orientation_pid( double kp, double ki, double kd ){
    robot_control.set_orientation_pid( kp, ki, kd );
}

void Shooter::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit
){
    robot_control.set_limits(
        translation_velocity_limit, rotation_velocity_limit
    );
}

void Shooter::go_to_shoot(
    const Vector2d & ball_position, 
    const Vector2d & robot_position,
    double robot_orientation,
    double time, double current_dt
){
    shooting_translation.position_robot = robot_position;
    shooting_translation.position_ball = ball_position;
    shooting_translation.goal_center = goal_center;
    shooting_translation.front_size = front_size;
    shooting_translation.radius_ball = radius_ball;

    shooting_rotation.orientation = robot_orientation;
    shooting_rotation.end = detail::vec2angle( goal_center - ball_position  );    

    robot_control.set_movement(
        shooting_translation,
        translation_velocity,
        translation_acceleration,
        shooting_rotation,
        angular_velocity,
        angular_acceleration,
        calculus_step, time, current_dt
    );
}

void Shooter::go_home(
    const Vector2d & ball_position, 
    const Vector2d & robot_position,
    double robot_orientation,
    double time, double current_dt
){
    home_translation.position_robot = robot_position;
    home_translation.position_home = ball_position;

    home_rotation.orientation = robot_orientation;
    home_rotation.position_ball = ball_position;
    home_rotation.position_robot = robot_position;

    robot_control.set_movement(
        shooting_translation,
        translation_velocity,
        translation_acceleration,
        shooting_rotation,
        angular_velocity,
        angular_acceleration,
        calculus_step, time, current_dt
    );
}


void Shooter::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now 
    //  this->robot_linear_position
    //  this->ball_position
    //  this->robot_angular_position 
    // are all avalaible

    if( birthday < 0 ){
        birthday = lastUpdate;
    }

    robot_control.update( time );
}

bool Shooter::is_static() const {
    return robot_control.is_static();
}

Control Shooter::control() const {
    if( age() <= 0.0 ) return Control::make_null();

    Control ctrl = robot_control.relative_control_in_robot_frame(
        robot_linear_position, robot_angular_position
    );

    return ctrl;
}




Vector2d Translation_for_shooting::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    Vector2d v = goal_center - position_ball;
    double distance = v.norm();
    v /= distance;
    Vector2d res = position_ball - (v * (front_size + radius_ball));
    
    Vector2d decalage(- norm_2(position_ball - position_robot) /2.0, 0.0);
    
    //SHOOT
    return  position_robot * (1.0-u) + res * u + (
        decalage * ( (2*u-1.0)*(2*u-1.0) - 1 )
    );
    //return  position_robot + Vector2d(u,u); 
//    
//    //RETOUR
//    //return  position_robot * (1.0-u) + Vector2d(-1.0, -1.0) * u 
//    //    ; //res * (1.0-u);
    //return  position_robot + Vector2d(u,0.0); 
    //return  position_robot; // + Vector2d(u,0.0); 
}

double Rotation_for_shooting::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    Angle a(rad2deg(orientation));
    Angle b(rad2deg(end));
    
    return  orientation + u * deg2rad( (b-a).getSignedValue() );
    //return  (M_PI/2.0)*u + orientation;
}

Vector2d Translation_for_home::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    return  position_robot * (1.0-u) + position_home * u;
}

double Rotation_for_home::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    double target = detail::vec2angle( position_ball - position_robot );
    //return  0.0*u + orientation;
    return  (1-u)*orientation + u*target;
}

}
}
