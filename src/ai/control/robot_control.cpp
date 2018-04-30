#include "robot_control.h"


#include <assert.h>
#include <math/curve.h>
#include "pid.h"
#include <math/matrix2d.h>

#define CALCULUS_ERROR 0.00000001

CurveForRobot::CurveForRobot(
    const std::function<Vector2d (double u)> & translation,
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
    // TODO Calculet un time_step plus précisement
    translation_movment(
        translation_curve, tranlsation_consign,
        calculus_step,
        calculus_error_for_translation        
    ),
    rotation_movment(
        angular_curve, angular_consign,
        calculus_step,
        calculus_error_for_rotation
    )
{ };

Vector2d CurveForRobot::translation(double t) const {
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
        Vector2d v = translation_movment(t);
        std::cout << "   {" << t << " : "  << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_translation_curve( double dt ) const {
    std::cout << "translation curve : " << std::endl;
    std::cout << "------------------- " << std::endl;
    for( double t=0; t<1.0; t += dt ){
        Vector2d v = translation_curve(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_rotation_movment( double dt ) const {
    std::cout << "rotation movment : " << std::endl;
    std::cout << "--------------------- " << std::endl;
    double max_time = rotation_movment.max_time();
    std::cout << "   max time : " << max_time;
    for( double t=0; t<max_time; t += dt ){
        Vector2d v = rotation_movment(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}

void CurveForRobot::print_rotation_curve( double dt ) const {
    std::cout << "rotation curve : " << std::endl;
    std::cout << "------------------- " << std::endl;
    for( double t=0; t<1.0; t += dt ){
        Vector2d v = angular_curve(t);
        std::cout << "   {" << t << " : " << v[0] << "," << v[1] << "}, " << std::endl; 
    }
}






void RobotControl::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit
){
    this->translation_velocity_limit = translation_velocity_limit;
    this->rotation_velocity_limit = ContinuousAngle(rotation_velocity_limit);
} 

PidControl RobotControl::limited_control(
    const Vector2d & robot_position, 
    const ContinuousAngle & robot_orientation
) const {
    PidControl res = absolute_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    if( rotation_velocity_limit > ContinuousAngle(0.0) ){ 
        if( res.velocity_rotation.abs() >= rotation_velocity_limit ){
            res.velocity_rotation *= (
                rotation_velocity_limit.value() / (
                    std::fabs( res.velocity_rotation.value() ) 
                ) 
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
    const Vector2d & robot_position, 
    const ContinuousAngle & robot_orientation
) const {
    PidControl res;

    Matrix2d rotation_matrix(
          std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
        - std::sin(robot_orientation.value()), std::cos(robot_orientation.value())
    );

    PidControl control = limited_control(
        robot_position, robot_orientation
    );

    res.velocity_translation = rotation_matrix * control.velocity_translation;
    res.velocity_rotation = control.velocity_rotation;

    return res;
}



/*
 * Calculus are explaine in the document : 
 * calcul_de_la_commande_en_vitesse_d_un_robot_holonome.org 
 */
PidControl RobotControl::absolute_to_relative_control(
    const PidControl & absolute_control,
    const Vector2d & robot_position, 
    const ContinuousAngle & robot_orientation,
    double dt
){
    PidControl res;
    Matrix2d rotation_matrix;

    const Vector2d & a_t = absolute_control.velocity_translation;
    const ContinuousAngle & a_r =  absolute_control.velocity_rotation;

    if( std::fabs(a_r.value()) > CALCULUS_ERROR ){
        rotation_matrix = Matrix2d(
            std::sin((a_r*dt+robot_orientation).value()) - std::sin(robot_orientation.value()), 
            std::cos((a_r*dt+robot_orientation).value()) - std::cos(robot_orientation.value()),
          - std::cos((a_r*dt+robot_orientation).value()) + std::cos(robot_orientation.value()), 
            std::sin((a_r*dt+robot_orientation).value()) - std::sin(robot_orientation.value())
        );
        rotation_matrix = (a_r*dt).value()*( rotation_matrix.inverse() );
    }else{
        rotation_matrix = Matrix2d(
            std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
          - std::sin(robot_orientation.value()), std::cos(robot_orientation.value())
        );
    }
    res.velocity_translation = rotation_matrix * a_t;
    res.velocity_rotation = a_r;

    return res;
}

PidControl RobotControl::relative_control_in_robot_frame(
    const Vector2d & robot_position, 
    const ContinuousAngle & robot_orientation
) const {
    return RobotControl::absolute_to_relative_control(
        limited_control( robot_position, robot_orientation ),
        robot_position, robot_orientation,
        get_dt()
    );
}

PidControl RobotControlWithPid::absolute_control_in_absolute_frame(
    const Vector2d & robot_position, 
    const ContinuousAngle & robot_orientation
) const {
    return PidController::absolute_control_in_absolute_frame(
        robot_position, robot_orientation
    );
}

double RobotControlWithPid::get_dt() const {
    return PidController::get_dt();
}

