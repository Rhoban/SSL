#include "defensor.h"
#include <math/tangents.h>
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {


Defensor::Defensor(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data),
    follower( Factory::fixed_consign_follower(ai_data) )
{
}

void Defensor::update(
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
    
    //int robot_id = 2;
    //const Robots_table & robot_table = ai_data.robots.at(Vision::Team::Ally);
    //const Ai::Robot & robot = robot_table.at(robot_id);
    
    const rhoban_geometry::Point & robot_position_point = robot.get_movement().linear_position( ai_data.time );
    

    rhoban_geometry::Point ally_goal_point = ally_goal_center();
    Vector2d direction = Vector2d(ally_goal_point) - Vector2d(ball_position());
    direction = direction/direction.norm();

    //Vector2d robot_ball_vector = robot_position_point - this->ball_position;

    //double target_rotation = std::atan2( robot_ball_vector.getY(), robot_ball_vector.getX() );
    double target_rotation = std::atan2( direction.getY(), direction.getX() );
    double target_radius_from_ball = 0.0;
    double error = 0.03;

    Vector2d target_position = Vector2d(ball_position()) + direction * (
         ai_data.constants.robot_radius + ai_data.constants.radius_ball + target_radius_from_ball + error
    );
    

    rhoban_geometry::Point waiting_position = rhoban_geometry::Point(0.0, 0.0) + ally_goal_point / 2 ;

    if(
        scalar_product(
            robot_position_point - Vector2d(ball_position()),
            target_position - Vector2d(ball_position())
        ) > 0
    ) {
        follower->avoid_the_ball(false);
    } else {
        follower->avoid_the_ball(true);
    }


    if ( ball_position().getX() < 0 ) {
        follower->set_following_position(target_position, target_rotation);
    } else {
        follower->set_following_position(waiting_position, target_rotation);
    }
    

    follower->update(time, robot, ball);   
}


Control Defensor::control() const {
    Control ctrl = follower->control();
    // ctrl.spin = true; // We active the dribler ! 
    return ctrl; 
}

Defensor::~Defensor(){
    delete follower;
}

}
}
