#include "robot_control_with_curve.h"

void RobotControlWithCurve::set_movement(
    const std::function<Eigen::Vector2d (double u)> & translation,
    double translation_velocity, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double angular_acceleration, 
    double calculus_step, double current_time
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
    init_time(current_time);
    set_static(false);
}

ContinuousAngle RobotControlWithCurve::goal_orientation( double t ) const {
    return curve.rotation(t);
}

Eigen::Vector2d RobotControlWithCurve::goal_position( double t ) const {
    return curve.translation(t);
}

RobotControlWithCurve::RobotControlWithCurve():
    curve(
        [](double t){ return Eigen::Vector2d(1.0,0.0); },
        0.0, 1.0, 
        [](double t){ return 1.0; },
        0.0, 1.0, 
        0.001
    )
{ }
