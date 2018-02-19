#ifndef __ROBOT_CONTROL_WITH_POSITION_FOLLOWING__H__
#define __ROBOT_CONTROL_WITH_POSITION_FOLLOWING__H__

#include "robot_control.h"

class RobotControlWithPositionFollowing : public RobotControlWithPid {
    protected:
        Eigen::Vector2d position;
        ContinuousAngle orientation;

    public:
        void set_goal(
            const Eigen::Vector2d & position, ContinuousAngle orientation
        );

        ContinuousAngle goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

#endif
