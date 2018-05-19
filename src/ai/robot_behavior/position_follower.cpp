#include "position_follower.h"

namespace RhobanSSL {
namespace Robot_behavior {



PositionFollower::PositionFollower(
    Ai::AiData & ai_data, double time, double dt
):
    ConsignFollower(ai_data),
    position(0.0, 0.0), angle(0.0)
{
    robot_control.init_time( time, dt );
} 

void PositionFollower::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & angle
){
    this->position = position_to_follow;
    this->angle = angle;
    this->angle = this->robot_angular_position;
    this->angle.set_to_nearest(angle); 
}

void PositionFollower::update_control(double time){
    robot_control.set_goal( position, angle );
    robot_control.update( time );
}

void PositionFollower::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now 
    //  this->robot_linear_position
    //  this->robot_angular_position 
    //  this->robot_linear_velocity
    //  this->robot_angular_velocity
    //  this->ball_position
    // are all avalaible

    update_control(time);
}

Control PositionFollower::control() const {
    Control ctrl = robot_control.limited_control(
        robot_linear_position, robot_angular_position,
        robot_linear_velocity, robot_angular_velocity
    );
    return ctrl;
}

void PositionFollower::set_translation_pid( double kp, double ki, double kd ){
    robot_control.set_translation_pid( kp, ki, kd );
}

void PositionFollower::set_orientation_pid( double kp, double ki, double kd ){
    robot_control.set_orientation_pid( kp, ki, kd );
}

void PositionFollower::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit,
    double translation_acceleration_limit,
    double rotation_acceleration_limit
){
    robot_control.set_limits(
        translation_velocity_limit, rotation_velocity_limit,
        translation_acceleration_limit, rotation_acceleration_limit
    );
}

}
}
