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
#include "annotations_ball_position.h"
#include <math/vector2d.h>

namespace RhobanSSL {
namespace Robot_behavior {
namespace Beginner {

Annotations_ball_position::Annotations_ball_position(
    Ai::AiData & ai_data
):
    RobotBehavior(ai_data)
{
}

void Annotations_ball_position::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    RobotBehavior::update_time_and_position(time, robot, ball);
    // Clear all annotations write before
    annotations.clear();

    // Add a cross on the ball.
    std::string color = "red";
    bool dash = false;
    annotations.addCross(ball_position(), color, dash);
}

Control Annotations_ball_position::control() const {
    return Control();
}

Annotations_ball_position::~Annotations_ball_position(){}

RhobanSSLAnnotation::Annotations Annotations_ball_position::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    annotations.addAnnotations( this->annotations );
    return annotations;
}
}
}
}
