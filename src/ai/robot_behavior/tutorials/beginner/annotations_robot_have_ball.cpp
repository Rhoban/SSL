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

#include "annotations_robot_have_ball.h"

namespace RhobanSSL {
namespace Robot_behavior {
namespace Beginner {

Robot_have_ball::Robot_have_ball(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data)
{
}

void Robot_have_ball::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    RobotBehavior::update_time_and_position( time, robot, ball );
    
    // Find the ally and the opponent closest to the ball
    int nb_ally_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Ally);
    int nb_opponent_closest_to_the_ball = get_shirt_number_of_closest_robot_to_the_ball(Vision::Opponent);
    
    // Get the robot ally and opponent.
    Ai::Robot ally_closest = get_robot(nb_ally_closest_to_the_ball, Vision::Ally);
    Ai::Robot opponent_closest = get_robot(nb_opponent_closest_to_the_ball, Vision::Opponent);

    // Find if the robot has the ball.
    int ally_have_ball = GameInformations::infra_red(nb_ally_closest_to_the_ball, Vision::Ally);
    int opponent_have_ball = GameInformations::infra_red(nb_opponent_closest_to_the_ball, Vision::Opponent);

    annotations.clear();
    
    std::string color_robot_have_ball = "blue";
    std::string color_robot_have_not_ball = "red";
    bool dash = false;
    // Find the robot that have the ball.
    if(opponent_have_ball ) {
        annotations.addCross(opponent_closest.get_movement().linear_position(ai_data.time), color_robot_have_ball, dash);
    } else if(ally_have_ball) {
        annotations.addCross(ally_closest.get_movement().linear_position(ai_data.time), color_robot_have_ball, dash);
    } else {
        annotations.addCross(ball_position(), color_robot_have_not_ball, dash);
    }
}

Control Robot_have_ball::control() const {
    return Control();
}

Robot_have_ball::~Robot_have_ball(){}

RhobanSSLAnnotation::Annotations Robot_have_ball::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    return annotations;
}

}
}
}
