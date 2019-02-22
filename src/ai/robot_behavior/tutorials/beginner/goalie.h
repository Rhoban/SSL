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

// REVIEW AB : Change BEGGINER to BEGINNER
#ifndef __ROBOT_BEHAVIOR__TUTORIALS__BEGGINER__GOALIE__
#define __ROBOT_BEHAVIOR__TUTORIALS__BEGGINER__GOALIE__

// REVIEW AB : Use absolute path instead of relative path.
#include "../../robot_behavior.h"
#include "../../factory.h"

namespace RhobanSSL
{

// REVIEW AB : Add an namespace beginner

namespace Robot_behavior {

/** Tutorial class to show how to place the robot behind the
 *  ball which aiming the center of ball.
 */

// REVIEW AB : Change the name of the class : Begginer_goalie -> Goalie
class Begginer_goalie : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    RhobanSSLAnnotation::Annotations annotations;

    public:
    Begginer_goalie(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

	virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Begginer_goalie();
};

};
}; //Namespace Rhoban

#endif
