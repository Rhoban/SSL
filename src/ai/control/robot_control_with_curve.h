#ifndef __ROBOT_CONTROL_WITH_CURVE__H__
#define __ROBOT_CONTROL_WITH_CURVE__H__

#include "robot_control.h"

class RobotControlWithCurve : public RobotControlWithPid {
    public:
        CurveForRobot curve;

        RobotControlWithCurve();

        void set_movement(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double translation_velocity, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double angular_acceleration, 
            double calculus_step, double current_time
        );

        ContinuousAngle goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

#endif
