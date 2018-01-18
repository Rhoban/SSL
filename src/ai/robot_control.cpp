#include "robot_control.h"


#include <assert.h>
#include "curve.h"
#include "pid.h"

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
    translation_movment(translation_curve, tranlsation_consign),
    rotation_movment(angular_curve, angular_consign)
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


RobotControlWithCurve::RobotControlWithCurve(): RobotControlWithCurve( 1.0, 0.0, 0.0) { }

RobotControlWithCurve::RobotControlWithCurve( double p, double i=0.0, double d=0.0 ):
    RobotControlWithCurve( p, i, d, p, i, d )
{ }

RobotControlWithCurve::RobotControlWithCurve(
    double p_t, double i_t, double d_t, 
    double p_o, double i_o, double d_o 
):
    curve(
        [](double t){ return Eigen::Vector2d(1.0,0.0); },
        0.0, 1.0, 
        [](double t){ return 1.0; },
        0.0, 1.0, 
        0.001
    ),
    PidControl( p_t, i_t, d_t, p_o, i_o, d_o )
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

/*
void Control::limits_the_contol(
    double rotation_velocity_limit, // 0.0 means no limit
    double translation_velocity_limit // 0.0 means no limit
){
    if( rotation_velocity_limit > 0.0 ){
        this->velocity_rotation = rotation_velocity_limit;
    }
    if( translation_velocity_limit > 0.0 ){
        this->velocity_translation /= translation_velocity_limit;
    }
}
*/
