#include <assert.h>
#include "movment.h"

#define CALCULUS_ERROR 0.000

VelocityConsign::VelocityConsign(
    double distance, 
    double max_velocity,
    double max_acceleration 
):
    distance(distance), max_velocity(max_velocity),
    max_acceleration(max_acceleration)
{ 
    assert(max_acceleration > 0.0); //Acceleration should be Greater than 0.
    if( not ( distance >= (2*max_velocity*max_velocity/max_acceleration) ) ){
        std::cerr << "Warning : Distance curve is too short for given maximal velocity and acceleration." << std::endl;
        std::cerr << "Warning :    we should have : distance > 2*max_velocity*max_velocity/max_acceleration" << std::endl;
        double old_velocity = max_velocity;
        max_velocity = std::sqrt( distance*max_acceleration/2.0 );
        std::cerr << "Warning :    we reduce the velocity from " << old_velocity << " to " << max_velocity << std::endl;
    }
}

double VelocityConsign::operator()(double t){
    double x = time_of_acceleration();
    double a = max_acceleration;
    double tm = time_of_deplacement(); 
    if( t<=0 ) return 0;
    if( t<= x/2.0 ) return a*t*t/x;
    if( t<= x ) return a*x/2 - a*(t-x)*(t-x)/x;
    if( t<= tm-x ) return a*x/2;
    if( t<= tm-x/2.0 ) return a*x/2 - a*(t-tm+x)*(t-tm+x)/x;
    if( t<= tm ) return a*(t-tm)*(t-tm)/x;
    return 0.0;
}
double VelocityConsign::time_of_deplacement(){
    double x = time_of_acceleration();
    double a = max_acceleration;
    return 2*distance/(a*x)+x;
}
double VelocityConsign::time_of_acceleration(){
    return 2*max_velocity/max_acceleration;
}


void Curve2d::init(){
    this->curve_length = arc_length( 1.0 );
}

Curve2d::Curve2d(
    const std::function<Eigen::Vector2d (double u)> & curve,
    double step_time
):curve(curve), step_time(step_time){
    init();
};

Curve2d::Curve2d( const Curve2d & curve ):
    curve(curve.curve), step_time(curve.step_time)
{
    init();
};

Eigen::Vector2d Curve2d::operator()(double u) const {
    return this->curve(u);
}

double Curve2d::size() const {
    return this->curve_length;
}

double Curve2d::arc_length( double u ) const {
    if( u<=0 ) return 0;
    if( u>1.0 ) return this->curve_length;

    double res = 0;
    Eigen::Vector2d old = curve( 0.0 );
    for( double v = 0.0; v <= u; v+=this->step_time ){
        Eigen::Vector2d current = curve( v );
        res += ( current - old ).norm();
        old = current;
    }
    Eigen::Vector2d current = curve( u );
    res += ( current - old ).norm();
    return res;
}

double Curve2d::inverse_of_arc_length( double l ) const {
    if( l<= 0 ) return 0.0;
    if( l>= this->curve_length ) return 1.0;
    double res = 0;
    Eigen::Vector2d old = curve( 0.0 );
    double v;
    for( v = 0.0; res < l; v+=this->step_time ){
        Eigen::Vector2d current = curve( v );
        res += ( current - old ).norm();
        old = current;
    }
    return v;
}




void RenormalizedCurve::init(){
    this->time_max = time(this->curve_length);
}

RenormalizedCurve::RenormalizedCurve(
    const std::function<Eigen::Vector2d (double u)> & curve,
    const std::function<double (double t)> & velocity_consign,
    double step_time
):Curve2d(curve, step_time), velocity_consign(velocity_consign){
    init();
};

RenormalizedCurve::RenormalizedCurve(
    const Curve2d & curve,
    const std::function<double (double t)> & velocity_consign
):Curve2d(curve), velocity_consign(velocity_consign){
    init();
};

double RenormalizedCurve::max_time() const {
    return this->time_max;
}

Eigen::Vector2d RenormalizedCurve::original_curve( double u ) const {
    return curve(u);
}

void RenormalizedCurve::set_step_time( double dt ){
    assert( dt>0.0 );
    this->step_time = dt;
    init();
}

double RenormalizedCurve::get_step_time( ) const {
    return this->step_time;
}

double RenormalizedCurve::position_consign( double t ) const {
    double res = 0.0;
    for(double u=0; u<t; u+=this->step_time){
        res += this->step_time * velocity_consign(u);
    } 
    return res;
}

double RenormalizedCurve::error_position_consign() const {
    double max_velocity = 0;
    for( double t = 0; t<max_time(); t+= this->step_time ){
        max_velocity = std::max( max_velocity, velocity_consign(t) );
    }
    return this->step_time * max_velocity;
}

double RenormalizedCurve::time( double length ) const {
    assert( 0<= length and length <= this->curve_length );
    double res = 0.0;
    double t;
    
    for( t=0.0; res < length; t+=this->step_time ){
        res += this->step_time * velocity_consign(t);
    } 
    return t;
}

Eigen::Vector2d RenormalizedCurve::operator()(double t) const {
    return original_curve( this->inverse_of_arc_length( position_consign(t) ) );
}






CurveForRobot::CurveForRobot(
    const std::function<Eigen::Vector2d (double u)> & translation,
    double translation_velocity, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double angular_acceleration, 
    double calculus_step
):
    rotation_fct(rotation),
    translation_curve( translation, calculus_step ),
    angular_curve(
        rotation_fct, calculus_step 
    ),
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

Eigen::Vector2d CurveForRobot::translation(double t){
    return translation_movment(t);
}
double CurveForRobot::rotation(double t){
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




RobotControl::RobotControl(): RobotControl( 1.0, 0.0, 0.0) { }

RobotControl::RobotControl( double p, double i=0.0, double d=0.0 ):
    RobotControl( p, i, d, p, i, d )
{ }

RobotControl::RobotControl(
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
    kp_t(p_t), ki_t(i_t), kd_t(d_t),
    kp_o(p_o), ki_o(i_o), kd_o(d_o),
    static_robot(true),
    time(0)
{ }

void RobotControl::set_static(){
    static_robot = false;
}
bool RobotControl::is_static() const {
    return static_robot;
}


void RobotControl::set_movment(
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
    start_time = current_time;
    time = 0.0;
    dt = 0.0;
    static_robot = false;
}

void RobotControl::set_orientation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_o = kp;
    this->ki_o = ki;
    this->kd_o = kd;
}

void RobotControl::set_translation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_t = kp;
    this->ki_t = ki;
    this->kd_t = kd;
}

void RobotControl::set_pid( double kp, double ki=0.0, double kd=0.0 ){
    set_orientation_pid( kp, ki, kd );
    set_translation_pid( kp, ki, kd );
}

void RobotControl::update(double current_time){
    this->dt = (current_time - start_time) - this->time;
    this->time = (current_time - start_time);
}

Eigen::Vector2d RobotControl::translation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){
    assert(dt>0);
    if( is_static() ) return Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d xt = curve.translation(time);
    Eigen::Vector2d xt_dt = curve.translation(time+dt);
    Eigen::Vector2d velocity = (xt_dt - xt )/dt;

    Eigen::Vector2d error = robot_position - xt;

    if( 
        std::abs( error[0] ) < CALCULUS_ERROR and 
        std::abs(error[1] ) < CALCULUS_ERROR
    ){
        error = Eigen::Vector2d(0.0,0.0);
    }

    Eigen::Vector2d absolute_command = (
        velocity - kp_t*error/dt - ki_t*error - kd_t*error/(dt*dt) 
    );

    return  absolute_command; 
}

double RobotControl::rotation_control_in_absolute_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){
    if( is_static() ) return 0.0;
    double theta_t = curve.rotation(time);
    double theta_t_dt = curve.rotation(time+dt);
    double velocity = (theta_t_dt - theta_t )/dt;
    double error = (
        std::fmod( robot_orientation - theta_t, M_PI )
    );

    if( std::abs( error ) <= CALCULUS_ERROR ){
        error = 0.0;
    }
    return (
        velocity - kp_o*error/dt - ki_o*error - kd_o*error/(dt*dt) 
    );
}

Control RobotControl::absolute_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){
    Control res;

    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 
          std::cos(robot_orientation), std::sin(robot_orientation),
        - std::sin(robot_orientation), std::cos(robot_orientation)
    ;

    Eigen::Vector2d absolute_translation = translation_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    double absolute_rotation = rotation_control_in_absolute_frame(
        robot_position, robot_orientation
    );

    res.velocity_translation = rotation_matrix * absolute_translation;
    res.velocity_rotation = absolute_rotation;

    return res;
}

Control RobotControl::relative_control_in_robot_frame(
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){

    Control res;
    Eigen::Matrix2d rotation_matrix;

    Eigen::Vector2d a_t = translation_control_in_absolute_frame(
        robot_position, robot_orientation
    );
    double a_r = rotation_control_in_absolute_frame(
        robot_position, robot_orientation
    );

    if( a_r != 0 ){
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
