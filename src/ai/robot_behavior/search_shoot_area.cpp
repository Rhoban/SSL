#include "search_shoot_area.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


SearchShootArea::SearchShootArea(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    random(rand()),
    follower( Factory::fixed_consign_follower(ai_data) )
{
  p1 = vector2point(
      Vector2d(
        oponent_corner_left() - rhoban_geometry::Point(1, 1)
      )
  );
  p2 = vector2point(
      Vector2d(
        center_mark() - rhoban_geometry::Point(-1, 2)
      )
  );
}

void SearchShootArea::update(
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

    annotations.clear();

    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( time );
    Vector2d ball_robot_vector = robot_position - ball_position();

    // Vector2d robot_goal_vector = robot_position - ball_position();
    int t = fmod(time,5);
    if(t == 0 ){
      random = rand();
    }

    srand(std::time(nullptr)); // use current time as seed for random generator
    rhoban_geometry::Point target_position = rhoban_geometry::Point(random%abs(p1.x - p2.x) + std::min(p1.x,p2.x) ,random%abs(p1.y - p2.y) + std::min(p1.y,p2.y));

    annotations.addCross( target_position.x, target_position.y );

    ContinuousAngle target_rotation = vector2angle( ball_robot_vector  );
    follower->avoid_the_ball(true);
    follower->set_following_position(target_position, target_rotation);
    follower->update(time, robot, ball);
}

Control SearchShootArea::control() const {
    Control ctrl = follower->control();
    return ctrl;
}

void SearchShootArea::declare_area( rhoban_geometry::Point p1, rhoban_geometry::Point p2){
  this->p1 = p1;
  this->p2 = p2;
}

SearchShootArea::~SearchShootArea(){
    delete follower;
}


RhobanSSLAnnotation::Annotations SearchShootArea::get_annotations() const {
        return annotations;
}

}
}
