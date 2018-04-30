#include "robot_control_with_position_following.h"

void RobotControlWithPositionFollowing::set_goal(
    const Vector2d & position, ContinuousAngle orientation
){
    this->position = position;
    this->orientation = orientation;
    set_static(false);
}

ContinuousAngle RobotControlWithPositionFollowing::goal_orientation( double t ) const {
    return orientation;
}

Vector2d RobotControlWithPositionFollowing::goal_position( double t ) const {
    return position;
}

