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

// REVIEW AB : Remove RADIUS_CIRCLE, it is not used.
#define RADIUS_CIRCLE 100
#include "robot_have_ball.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {

Begginer_robot_have_ball::Begginer_robot_have_ball(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data)
{
}

void Begginer_robot_have_ball::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    // Find the ally and the opponent closest to the ball
    // REVIEW AB : The name of the variable is not well choosen :
    //  nb_ally_closest_to_the_ball -> shirt_number_of_closest_ally_to_the_ball
    int nb_ally_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Ally);
    // REVIEW AB : The name of the variable is not well choosen :
    //  nb_opponent_closest_to_the_ball -> shirt_number_of_closest_opponent_to_the_ball
    int nb_opponent_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Opponent);
    
    // Get the robot ally and opponent.
    // REVIEW AB : 
    //   ally_closest -> closet_ally
    Ai::Robot ally_closest = get_robot(nb_ally_closest_to_the_ball, Vision::Ally);
    // REVIEW AB : 
    //   opponent_closest -> closest_opponent
    Ai::Robot opponent_closest = get_robot(nb_opponent_closest_to_the_ball, Vision::Opponent);

    // Find if the robot has the ball.
    int ally_have_ball = GameInformations::infra_red(nb_ally_closest_to_the_ball, Vision::Ally);
    // REVIEW AB : This is not possible to have this information ! 
    // We need to implement an algorithme that detect that an opponent robot 
    // have a ball.
    // REVIEW AB : use a bool instead of an int.
    int opponent_have_ball = GameInformations::infra_red(nb_opponent_closest_to_the_ball, Vision::Opponent);

    annotations.clear();
    
    // REVIEW AB : Don't use magic number, for the parameters.
    //  Use varaible : 
    //  std::string robot_color = "blue"; 
    //  std::string ball_color = "red"; 
    //  std::string bool dashed = false;
    
    // Find the robot that have the ball.
    if(opponent_have_ball ) {
        annotations.addCross(opponent_closest.get_movement().linear_position(ai_data.time), "blue", false);
    } else if(ally_have_ball) {
        annotations.addCross(ally_closest.get_movement().linear_position(ai_data.time), "blue", false);
    } else {
        annotations.addCross(ball_position(), "red", false);
    }
 
    const rhoban_geometry::Point & robot_position = robot.get_movement().linear_position( ai_data.time );
}

Control Begginer_robot_have_ball::control() const {
    return Control();
}

Begginer_robot_have_ball::~Begginer_robot_have_ball(){}

RhobanSSLAnnotation::Annotations Begginer_robot_have_ball::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    return annotations;
}

}
}
