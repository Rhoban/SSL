#include "robot_behavior.h"
#include <math/vector2d.h>

namespace RhobanSSL {


Control::Control():
    PidControl(), kick(false), active(true), ignore(false)
{ }


Control::Control(bool kick, bool active, bool ignore):
    kick(kick), active(active), ignore(ignore)
{ }

Control::Control(const PidControl & c):
    PidControl(c), kick(false), active(true), ignore(false)
{ }

std::ostream& operator << ( std::ostream & out, const Control& control  ){
    out << "{ctrl : " << static_cast<PidControl>(control)
        << ", kick : " << control.kick << ", acitve : " << control.active << ", ignore : " << control.ignore <<"}";
    return out;
}

Control Control::make_null(){
    return Control(false, true, false);
}

Control Control::make_desactivated(){
    return Control(false, false, false);
}

Control Control::make_ignored(){
    return Control(false, false, true);
}



namespace Robot_behavior {

namespace detail {

double vec2angle( Vector2d direction ){
    double norm = direction.norm();
    if( norm == 0.0 ) return 0.0;
    direction /= norm;
    double res = std::acos( direction[0] );
    if( direction[1] <= 0 ) return -res;
    return res;
}

}


RobotBehavior::RobotBehavior( Ai::AiData & ai_data ) :
    GameInformations(ai_data),
    birthday(-1.0), ai_data(ai_data)
{ };

double RobotBehavior::age() const { return lastUpdate - birthday; };
bool RobotBehavior::is_born() const { return birthday > 0; };
void RobotBehavior::set_birthday( double birthday ){
    assert( birthday > 0 );
    this->birthday = birthday;
};

void RobotBehavior::update_time_and_position(
    double time, 
    const Ai::Robot & robot, const Ai::Ball & ball
){
    this->robot_ptr = &robot;
    this->ball_ptr = &ball; 
    lastUpdate = time;
    this->robot_linear_position = Vector2d(
        robot.get_movement().linear_position(time)
    );
    this->robot_angular_position = robot.get_movement().angular_position(
        time
    );
    this->robot_linear_velocity = robot.get_movement().linear_velocity(time);
    this->robot_angular_velocity = robot.get_movement().angular_velocity(time);
};

const Ai::Robot & RobotBehavior::robot() const {
    return *robot_ptr;
}

const Ai::Ball & RobotBehavior::ball() const {
    return *ball_ptr;
}

rhoban_geometry::Point RobotBehavior::ball_position() const {
    return ball().get_movement().linear_position(time());
}

}
}
