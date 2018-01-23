#include "robot_control.h"


#include <assert.h>
#include "curve.h"
#include "pid.h"

#define CALCULUS_ERROR 0.00000001

CurveForRobot::CurveForRobot(
    const std::function<Eigen::Vector2d (double u)> & translation,
    double translation_velocity, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double angular_acceleration, 
    double calculus_step
):
    rotation_fct(rotation),
    translation_curve( translation, calculus_step ),
    angular_curve( rotation_fct, calculus_step ),
    tranlsation_consign(
        translation_curve.size(),
        translation_velocity, translation_acceleration 
    ),
    angular_consign(
        angular_curve.size(), 
        angular_velocity, angular_acceleration 
    ),
    calculus_error_for_translation(
        std::min(
            0.0001, // 0.1 mm (in meter)
            calculus_step/100
        )
    ),
    calculus_error_for_rotation(
        std::min(
            0.0001, // approx. 1/100 degree (in radian)
            calculus_step/100
        )
    ),
    translation_movment(
        translation_curve, tranlsation_consign,
        calculus_error_for_translation        
    ),
    rotation_movment(
        angular_curve, angular_consign,
        calculus_error_for_rotation
    )
{ };

Eigen::Vector2d CurveForRobot::translation(double t) const {
    return translation_movment(t);
}
double CurveForRobot::rotation(double t) const {
    return rotation_movment(t)[0];
}

void CurveForRobot::print_translation_movment( double dt ) const {
    std::cout << "translation movment : " << std::endl;
    std::cout << "--------------------- " << std::endl;
    double max_time = translation_movment.max_time();
    std::cout << "   max time : " << max_time;
    for( double t=0; t<max_time; t += dt ){
        Eigen::Vector2d v = translation_movment(t);
        std::cout << "   {" << t << " : "  << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_translation_curve( double dt ) const {
    std::cout << "translation curve : " << std::endl;
    std::cout << "------------------- " << std::endl;
    for( double t=0; t<1.0; t += dt ){
        Eigen::Vector2d v = translation_curve(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_rotation_movment( double dt ) const {
    std::cout << "rotation movment : " << std::endl;
    std::cout << "--------------------- " << std::endl;
    double max_time = rotation_movment.max_time();
    std::cout << "   max time : " << max_time;
    for( double t=0; t<max_time; t += dt ){
        Eigen::Vector2d v = rotation_movment(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_rotation_curve( double dt ) const {
    std::cout << "rotation curve : " << std::endl;
    std::cout << "------------------- " << std::endl;
    for( double t=0; t<1.0; t += dt ){
        Eigen::Vector2d v = angular_curve(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}






void RobotControlWithCurve::set_movment(
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

double RobotControlWithCurve::goal_orientation( double t ) const {
    return curve.rotation(t);
}

Eigen::Vector2d RobotControlWithCurve::goal_position( double t ) const {
    return curve.translation(t);
}

void RobotControl::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit
){
    this->translation_velocity_limit = translation_velocity_limit;
    this->rotation_velocity_limit = rotation_velocity_limit;
} 

PidControl RobotControl::limited_control(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {
    PidControl res = absolute_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    if( rotation_velocity_limit > 0.0 ){ 
        if( std::abs( res.velocity_rotation ) >= rotation_velocity_limit ){
            res.velocity_rotation *= (
                rotation_velocity_limit / std::abs( res.velocity_rotation ) 
            );
            std::cerr << "We limit the rotation velocity to" << 
                res.velocity_rotation << "!" << std::endl;
        }
    }

    if( translation_velocity_limit > 0.0 ){
        if( res.velocity_translation.norm() >= translation_velocity_limit ){
            res.velocity_translation *= ( 
                translation_velocity_limit / res.velocity_translation.norm()
            );
            std::cerr << "We limit the translation velocity to" << 
                res.velocity_translation << "!" << std::endl;
        }
    }
    return res;
}

PidControl RobotControl::absolute_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {
    PidControl res;

    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 
          std::cos(robot_orientation), std::sin(robot_orientation),
        - std::sin(robot_orientation), std::cos(robot_orientation)
    ;

    PidControl control = limited_control(
        robot_position, robot_orientation
    );

    res.velocity_translation = rotation_matrix * control.velocity_translation;
    res.velocity_rotation = control.velocity_rotation;

    return res;
}

PidControl RobotControl::relative_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {

    PidControl res;
    Eigen::Matrix2d rotation_matrix;

    PidControl absolute_control = limited_control(
        robot_position, robot_orientation
    );
    Eigen::Vector2d & a_t = absolute_control.velocity_translation;
    double a_r =  absolute_control.velocity_rotation;
    double dt = get_dt();

    if( std::abs(a_r) > CALCULUS_ERROR ){
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



PidControl RobotControlWithPid::absolute_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
) const {
    return PidController::absolute_control_in_absolute_frame(
        robot_position, robot_orientation
    );
}

double RobotControlWithPid::get_dt() const {
    return PidController::get_dt();
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





void RobotControlWithPositionFollowing::set_goal(
    const Eigen::Vector2d & position, double orientation
){
    this->position = position;
    this->orientation = orientation;
    set_static(false);
}


double RobotControlWithPositionFollowing::goal_orientation( double t ) const {
    return orientation;
}

Eigen::Vector2d RobotControlWithPositionFollowing::goal_position( double t ) const {
    return position;
}

