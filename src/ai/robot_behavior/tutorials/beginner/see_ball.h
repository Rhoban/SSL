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

// REVIEW AB : BEGGINER --> BEGINNER 
#ifndef __ROBOT_BEHAVIOR__BEGGINER__SEE_BALL__
#define __ROBOT_BEHAVIOR__BEGGINER__SEE_BALL__

// REVIEW AB : Mettre des chemins absolus avec <>
#include "../../robot_behavior.h"
// REVIEW AB : Mettre des chemins absolus avec <>
#include "../../factory.h"


namespace RhobanSSL
{
namespace Robot_behavior {

// REVIEW AB : mettre un namespace Beginner 


// REVIEW AB : renomer partout Begginer_see_ball à See_ball
class Begginer_see_ball : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    RhobanSSLAnnotation::Annotations annotations;

    public:
    Begginer_see_ball(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

	virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Begginer_see_ball();
};

};
}; //Namespace Rhoban

#endif
