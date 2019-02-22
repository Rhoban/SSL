/*
    This file is part of SSL.
    
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#define RADIUS_CIRCLE 100
#include "robot_near_ball.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

Begginer_robot_near_ball::Begginer_robot_near_ball(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data)
{
}

void Begginer_robot_near_ball::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    RobotBehavior::update_time_and_position( time, robot, ball );
    // Find the ally and the opponent closest to the ball

    // REVIEW AB : Change the name of the variable 
    //   nb_ally_closest_to_the_ball 
    //   ->
    //   ally_shirt_number
Change the name of the variable 
    int nb_ally_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Ally);
    // REVIEW AB : Change the name of the variable 
    //   nb_opponent_closest_to_the_ball
    //   ->
    //   opponent_shirt_number
    int nb_opponent_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Opponent);
    
    // Get the robot ally and opponent.
    // REVIEW AB : ally_closest --> closest_ally
    Ai::Robot ally_closest = get_robot(nb_ally_closest_to_the_ball, Vision::Ally);
    // REVIEW AB : opponent_closest --> closest_opponent
    Ai::Robot opponent_closest = get_robot(nb_opponent_closest_to_the_ball, Vision::Opponent);

    // Create the vector between the robots and the ball.
    Vector2d vec_ally_to_ball = ball_position() - ally_closest.get_movement().linear_position( ai_data.time ); 
    Vector2d vec_opponent_to_ball = ball_position() - opponent_closest.get_movement().linear_position(ai_data.time);

    // Find the distance between them and the ball.
    double dist_ally = vec_ally_to_ball.norm();
    double dist_opponent = vec_opponent_to_ball.norm();

    annotations.clear();

    // REVIEW AB : Don't use magic number ! 
    //    std::string opponnent_color = "blue";
    //    std::string ally_color = "blue";
    //    bool dashed = false;
    // Search the nearest robot between the ally and the opponent.
    if(dist_ally > dist_opponent) {
        annotations.addCross(opponent_closest.get_movement().linear_position(ai_data.time), "blue", false);
    } else if(dist_ally < dist_opponent) {
        annotations.addCross(ally_closest.get_movement().linear_position(ai_data.time), "blue", false);
    } else {
        annotations.addCross(opponent_closest.get_movement().linear_position(ai_data.time), "blue", false);
        annotations.addCross(ally_closest.get_movement().linear_position(ai_data.time), "blue", false);
    }
 
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
}

Control Begginer_robot_near_ball::control() const {
    return Control();
}

Begginer_robot_near_ball::~Begginer_robot_near_ball(){}

RhobanSSLAnnotation::Annotations Begginer_robot_near_ball::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    return annotations;
}

}
}
