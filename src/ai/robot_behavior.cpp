#include "robot_behavior.h"

double angle( Eigen::Vector2d direction ){
    double norm = direction.norm();
    if( norm == 0.0 ) return 0.0;
    direction /= norm;
    double res = std::acos( direction[0] );
    if( direction[1] <= 0 ) return -res;
    return res;
}

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
    double goalie_radius
){
    this->left_post_position = left_post_position;
    this->right_post_position = right_post_position;
    this->waiting_goal_position = waiting_goal_position;
    this->goal_center = (left_post_position + right_post_position)/2.0;
    this->goalie_radius = goalie_radius;
}

void Goalie::update(
    const Eigen::Vector2d & ball_position, 
    const Eigen::Vector2d & robot_position,
    double robot_orientation,
    double time
) {
    this->robot_position = robot_position;
    this->robot_orientation = robot_orientation;

    double goal_rotation = angle(ball_position - robot_position);

    Eigen::Vector2d defender_pos = calculate_goal_position(
        ball_position, right_post_position, left_post_position,
        goalie_radius
    );
    
    double rayon_surface_reparation = 0.75;
    if( (defender_pos - goal_center).norm() > rayon_surface_reparation ){
        defender_pos = waiting_goal_position;
    }
    
    robot_control.set_goal(
        defender_pos, goal_rotation
    );

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




Shooter::Shooter(){ } 

void Shooter::set_translation_pid( double kp, double ki, double kd ){
    robot_control.set_translation_pid( kp, ki, kd );
}

void Shooter::set_orientation_pid( double kp, double ki, double kd ){
    robot_control.set_orientation_pid( kp, ki, kd );
}

void Shooter::init(
    const Eigen::Vector2d & goal_center, double robot_radius,
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

    shooting_rotation.orientation = robot_orientation;
    shooting_rotation.end = angle( goal_center - ball_position  );    

    robot_control.set_movment(
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

    robot_control.set_movment(
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
    const Eigen::Vector2d & ball_position, 
    const Eigen::Vector2d & robot_position,
    double robot_orientation,
    double time
){
    this->robot_position = robot_position;
    this->robot_orientation = robot_orientation;
    this->ball_position = ball_position;
    robot_control.update( time );
}

bool Shooter::is_static() const {
    return robot_control.is_static();
}

Control Shooter::control() const {
    Control ctrl = robot_control.relative_control_in_robot_frame(
        robot_position, robot_orientation
    );
    
    if( (ball_position - robot_position ).norm() < 0.068 ){
        ctrl.kick = true;
    }
    return ctrl;
}






Eigen::Vector2d Translation_for_shooting::operator()(double u) const {
//    Eigen::Vector2d v = goal_center - position_ball;
//    v /= v.norm();
//    Eigen::Vector2d n( -v[1], v[0] );
//    double al;
//    double z = 0.0;
//    if( n[0] >= 0){
//        al=z;
//    }else{
//        al=-z;
//    }
//    Eigen::Vector2d res = position_ball - (v * 0.03);
//    
//    //SHOOT
//    return  position_robot * (1.0-u) + res * u + 
//        ( ( u < .5 ) ?  n*(al*u) : n*( (1.0-u)*al*u) )
//        ; //res * (1.0-u);
//    //return  position_robot + Eigen::Vector2d(u,u); 
//    
//    //RETOUR
//    //return  position_robot * (1.0-u) + Eigen::Vector2d(-1.0, -1.0) * u 
//    //    ; //res * (1.0-u);
    return  position_robot + Eigen::Vector2d(u,0.0); 
    //return  position_robot; // + Eigen::Vector2d(u,0.0); 
};

double Rotation_for_shooting::operator()(double u) const {
    //Angle a(rad2deg(orientation));
    //Angle b(rad2deg(end));
    //return  deg2rad(Angle::weightedAverage(a,1-u,b,u).getSignedValue());
    //return  (M_PI/2.0)*u + orientation;
    return  (M_PI/2.0)*u + orientation;
};

Eigen::Vector2d Translation_for_home::operator()(double u) const {
    return  position_robot * (1.0-u) + position_home * u;
};

double Rotation_for_home::operator()(double u) const {
    double target = angle( position_ball - position_robot );
    //return  0.0*u + orientation;
    return  (1-u)*orientation + u*target;
};


