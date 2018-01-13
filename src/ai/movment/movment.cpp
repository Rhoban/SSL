#include <assert.h>
#include "movment.h"


VelocityConsign::VelocityConsign(
    double distance, double max_acceleration, 
    double max_velocity
):
    distance(distance), max_acceleration(max_acceleration),
    max_velocity(max_velocity)
{ }

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
    double angular_acceleration, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double translation_velocity,
    double calculus_step
):
    translation_curve( translation, calculus_step ),
    angular_curve(
        [&](double u){ return Eigen::Vector2d(rotation(u),0); },
        calculus_step
    ),
    tranlsation_consign(
        translation_curve.size(),
        translation_acceleration, translation_velocity
    ),
    angular_consign(
        angular_curve.size(), angular_acceleration, angular_velocity
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





RobotControl::RobotControl(
    const std::function<Eigen::Vector2d (double u)> & translation,
    double angular_acceleration, double translation_acceleration,
    const std::function<double (double u)> & rotation,
    double angular_velocity, double translation_velocity,
    double calculus_step
):
    curve(
        translation,
        angular_acceleration, translation_acceleration,
        rotation,
        angular_velocity, translation_velocity,
        calculus_step
    )
{ }

void RobotControl::set_orientation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_o = kp;
    this->ki_o = ki;
    this->kd_o = kd;
}

void RobotControl::set_tranlsation_pid( double kp, double ki=0.0, double kd=0.0 ){
    this->kp_t = kp;
    this->ki_t = ki;
    this->kd_t = kd;
}

void RobotControl::set_pid( double kp, double ki=0.0, double kd=0.0 ){
    set_orientation_pid( kp, ki, kd );
    set_tranlsation_pid( kp, ki, kd );
}

Eigen::Vector2d RobotControl::translation_command(
    double t, double dt,
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){
    Eigen::Vector2d xt = curve.translation(t);
    Eigen::Vector2d xt_dt = curve.translation(t+dt);
    Eigen::Vector2d velocity = (xt_dt - xt )/dt;
    Eigen::Vector2d error = robot_position - xt;

    Eigen::Vector2d absolute_command = (
        dt*velocity - kp_t*error - ki_t*error*dt - kd_t*error/dt 
        + robot_position
    );

    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << 
          std::cos(robot_orientation), std::sin(robot_orientation),
        - std::sin(robot_orientation), std::cos(robot_orientation)
    ;

    return rotation_matrix * absolute_command; 
}

double RobotControl::rotation_command(
    double t, double dt,
    const Eigen::Vector2d & robot_position, 
    double robot_orientation
){
    double theta_t = curve.rotation(t);
    double theta_t_dt = curve.rotation(t+dt);
    double velocity = (theta_t_dt - theta_t )/dt;
    double error = robot_orientation - theta_t;
    return (
        dt*velocity - kp_o*error - ki_o*error*dt - kd_o*error/dt 
        + robot_orientation
    );
}


