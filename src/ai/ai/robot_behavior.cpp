#include "robot_behavior.h"

namespace RhobanSSL {

Control::Control():
    kick(false), active(true), ignore(false)
{ }


Control::Control(bool kick, bool active, bool ignore):
    kick(kick), active(active), ignore(ignore)
{ }

Control::Control(const PidControl & c):
    PidControl(c), kick(false), active(true), ignore(ignore)
{ }


Control Control::make_null(){
    return Control(false, true, false);
}

Control Control::make_desactived(){
    return Control(false, false, true);
}

Control Control::make_ignored(){
    return Control(false, false, true);
}

double angle( Eigen::Vector2d direction ){
    double norm = direction.norm();
    if( norm == 0.0 ) return 0.0;
    direction /= norm;
    double res = std::acos( direction[0] );
    if( direction[1] <= 0 ) return -res;
    return res;
}


RobotBehavior::RobotBehavior() : birthday(-1.0) { };

double RobotBehavior::age() const { return lastUpdate - birthday; };
bool RobotBehavior::is_born() const { return birthday > 0; };
double RobotBehavior::set_birthday( double birthday ){
    assert( birthday > 0 );
    this->birthday = birthday;
};

void RobotBehavior::update(
    double time, 
    const Ai::Robot & robot, const Ai::Ball & ball
){
    lastUpdate = time;
    this->robot_position = Eigen::Vector2d( 
        robot.get_movement().linear_position(time).getX(), 
        robot.get_movement().linear_position(time).getY()
    );
    const Movement & mov = robot.get_movement();
    this->ball_position = Eigen::Vector2d(
        ball.get_movement().linear_position(time).getX(),
        ball.get_movement().linear_position(time).getY()
    );
    this->robot_orientation = robot.get_movement().angular_position(
        time
    ).getSignedValue();
};




Eigen::Vector2d Goalie::calculate_goal_position(
    const Eigen::Vector2d & ball_position,
    const Eigen::Vector2d & poteau_droit,
    const Eigen::Vector2d & poteau_gauche,
    double goalie_radius
){
    Eigen::Vector2d R = poteau_droit - ball_position;
    R /= R.norm();
    Eigen::Vector2d L = poteau_gauche - ball_position;
    L /= L.norm();
    Eigen::Matrix2d m;
    m << 
        -R[1], R[0],
         L[1], -L[0];
    return (
        ball_position + 
        m.inverse() * Eigen::Vector2d(goalie_radius, goalie_radius)
    );
}


Goalie::Goalie(
){ } 

void Goalie::init(
    const Eigen::Vector2d & left_post_position,
    const Eigen::Vector2d & right_post_position,
    const Eigen::Vector2d & waiting_goal_position,
    double penalty_rayon,
    double goalie_radius
){
    this->left_post_position = left_post_position;
    this->right_post_position = right_post_position;
    this->waiting_goal_position = waiting_goal_position;
    this->goal_center = (left_post_position + right_post_position)/2.0;
    this->penalty_rayon = penalty_rayon;
    this->goalie_radius = goalie_radius;
}

void Goalie::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
) {
    RobotBehavior::update(time, robot, ball);

    double goal_rotation = angle(ball_position - robot_position);


    Eigen::Vector2d defender_pos = calculate_goal_position(
        ball_position, right_post_position, left_post_position,
        goalie_radius
    );

    if( (defender_pos - goal_center).norm() > penalty_rayon ){
        defender_pos = waiting_goal_position;
    }

    robot_control.set_goal( defender_pos, goal_rotation );
    robot_control.update( time );
}

Control Goalie::control() const {
    Control ctrl = robot_control.relative_control_in_robot_frame(
        robot_position, robot_orientation
    );

    return ctrl;
}

void Goalie::set_translation_pid( double kp, double ki, double kd ){
    robot_control.set_translation_pid( kp, ki, kd );
}

void Goalie::set_orientation_pid( double kp, double ki, double kd ){
    robot_control.set_orientation_pid( kp, ki, kd );
}

void Goalie::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit
){
    robot_control.set_limits(
        translation_velocity_limit, rotation_velocity_limit
    );
}



Shooter::Shooter(){ } 

void Shooter::set_translation_pid( double kp, double ki, double kd ){
    robot_control.set_translation_pid( kp, ki, kd );
}

void Shooter::set_orientation_pid( double kp, double ki, double kd ){
    robot_control.set_orientation_pid( kp, ki, kd );
}

void Shooter::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit
){
    robot_control.set_limits(
        translation_velocity_limit, rotation_velocity_limit
    );
}

void Shooter::init(
    const Eigen::Vector2d & goal_center, double robot_radius,
    double front_size, double radius_ball,
    double translation_velocity,
    double translation_acceleration,
    double angular_velocity,
    double angular_acceleration,
    double calculus_step
){
    this->goal_center = goal_center;
    this->robot_radius = robot_radius;
    this->translation_velocity = translation_velocity;
    this->translation_acceleration = translation_acceleration;
    this->angular_velocity = angular_velocity;
    this->angular_acceleration = angular_acceleration;
    this->calculus_step = calculus_step;
    this->front_size = front_size;
    this->radius_ball = radius_ball;
}

void Shooter::go_to_shoot(
    const Eigen::Vector2d & ball_position, 
    const Eigen::Vector2d & robot_position,
    double robot_orientation,
    double time
){
    shooting_translation.position_robot = robot_position;
    shooting_translation.position_ball = ball_position;
    shooting_translation.goal_center = goal_center;
    shooting_translation.front_size = front_size;
    shooting_translation.radius_ball = radius_ball;

    shooting_rotation.orientation = robot_orientation;
    shooting_rotation.end = angle( goal_center - ball_position  );    

    robot_control.set_movement(
        shooting_translation,
        translation_velocity,
        translation_acceleration,
        shooting_rotation,
        angular_velocity,
        angular_acceleration,
        calculus_step, time 
    );
}

void Shooter::go_home(
    const Eigen::Vector2d & ball_position, 
    const Eigen::Vector2d & robot_position,
    double robot_orientation,
    double time
){
    home_translation.position_robot = robot_position;
    home_translation.position_home = ball_position;

    home_rotation.orientation = robot_orientation;
    home_rotation.position_ball = ball_position;
    home_rotation.position_robot = robot_position;

    robot_control.set_movement(
        shooting_translation,
        translation_velocity,
        translation_acceleration,
        shooting_rotation,
        angular_velocity,
        angular_acceleration,
        calculus_step, time 
    );
}


void Shooter::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    RobotBehavior::update(time, robot, ball);

    if( birthday < 0 ){
        birthday = lastUpdate;
        go_to_shoot( ball_position, robot_position, robot_orientation, time );
    }

    robot_control.update( time );
}

bool Shooter::is_static() const {
    return robot_control.is_static();
}

Control Shooter::control() const {
    if( age() <= 0.0 ) return Control::make_null();

    Control ctrl = robot_control.relative_control_in_robot_frame(
        robot_position, robot_orientation
    );

    return ctrl;
}

Eigen::Vector2d Translation_for_shooting::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    Eigen::Vector2d v = goal_center - position_ball;
    double distance = v.norm();
    v /= distance;
    Eigen::Vector2d res = position_ball - (v * (front_size + radius_ball));
    
    Eigen::Vector2d decalage(- (position_ball - position_robot).norm() /2.0, 0.0);
    
    //SHOOT
    return  position_robot * (1.0-u) + res * u + (
        decalage * ( (2*u-1.0)*(2*u-1.0) - 1 )
    );
    //return  position_robot + Eigen::Vector2d(u,u); 
//    
//    //RETOUR
//    //return  position_robot * (1.0-u) + Eigen::Vector2d(-1.0, -1.0) * u 
//    //    ; //res * (1.0-u);
    //return  position_robot + Eigen::Vector2d(u,0.0); 
    //return  position_robot; // + Eigen::Vector2d(u,0.0); 
}

double Rotation_for_shooting::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    Angle a(rad2deg(orientation));
    Angle b(rad2deg(end));
    
    return  orientation + u * deg2rad( (b-a).getSignedValue() );
    //return  (M_PI/2.0)*u + orientation;
}

Eigen::Vector2d Translation_for_home::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    return  position_robot * (1.0-u) + position_home * u;
}

double Rotation_for_home::operator()(double u) const {
    if( u <= 0.0 ){
        u = 0.0;
    }
    if( u >= 1.0 ){
        u = 1.0;
    }
    double target = angle( position_ball - position_robot );
    //return  0.0*u + orientation;
    return  (1-u)*orientation + u*target;
}

}
