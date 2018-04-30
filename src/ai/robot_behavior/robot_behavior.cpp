#include "robot_behavior.h"
#include <math/eigen_convertion.h>

namespace RhobanSSL {

namespace detail {

double vec2angle( Eigen::Vector2d direction ){
    double norm = direction.norm();
    if( norm == 0.0 ) return 0.0;
    direction /= norm;
    double res = std::acos( direction[0] );
    if( direction[1] <= 0 ) return -res;
    return res;
}

}

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







RobotBehavior::RobotBehavior() : birthday(-1.0) { };

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
    lastUpdate = time;
    this->robot_linear_position = point2eigen(
        robot.get_movement().linear_position(time)
    );
    this->ball_position = vector2eigen( 
        ball.get_movement().linear_position(time)
    );
    this->robot_angular_position = robot.get_movement().angular_position(
        time
    );
};

}
