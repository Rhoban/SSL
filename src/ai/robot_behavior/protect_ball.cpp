#include "protect_ball.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


ProtectBall::ProtectBall(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    radius(-1),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void ProtectBall::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Now
    //  this->robot_linear_position
    //  this->robot_angular_position
    // are all avalaible
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time );
    const rhoban_geometry::Point & oponent_goal_point = oponent_goal_center();

    Vector2d ball_goal_vector = oponent_goal_point - ball_position();
    Vector2d ball_robot_vector = robot_position - ball_position();
    Vector2d target_position;
    if(ball_goal_vector.norm() > radius){
      target_position = ball_position();
    }else{
      target_position = ball_position() + radius*Vector2d(cos( time ), sin( time ));
    }

    double target_rotation = detail::vec2angle(ball_goal_vector);

    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control ProtectBall::control() const {
    Control ctrl = follower->control();
    return ctrl;
}

void ProtectBall::declare_radius( double radius ){
    this->radius = radius;
}

ProtectBall::~ProtectBall(){
    delete follower;
}

}
}