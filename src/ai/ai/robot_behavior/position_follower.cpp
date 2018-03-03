#include "position_follower.h"

namespace RhobanSSL {



PositionFollower::PositionFollower(double time, double dt):
    position(0.0, 0.0), angle(0.0)
{
    robot_control.init_time( time, dt );
} 

void PositionFollower::set_following_position(
    const Eigen::Vector2d & position_to_follow,
    const ContinuousAngle & angle
){
    this->position = position_to_follow;
    this->angle = angle;
}

void PositionFollower::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
) {
    RobotBehavior::update(time, robot, ball);

    robot_control.set_goal( position, angle );
    robot_control.update( time );
}

Control PositionFollower::control() const {
    Control ctrl = robot_control.relative_control_in_robot_frame(
        robot_position, robot_orientation
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
    double rotation_velocity_limit
){
    robot_control.set_limits(
        translation_velocity_limit, rotation_velocity_limit
    );
}


}
