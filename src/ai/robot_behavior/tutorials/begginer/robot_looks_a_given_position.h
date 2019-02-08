/*
    This file is part of SSL.

    Copyright 2019 Beltran Lila (lila.beltran@etu.u-bordeaux.fr)

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

#ifndef __ROBOT_BEHAVIOR__TUTORIALS__ROBOT__LOOKS__A__GIVEN__POSITION
#define __ROBOT_BEHAVIOR__TUTORIALS__ROBOT__LOOKS__A__GIVEN__POSITION

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>

namespace RhobanSSL
{
namespace Robot_behavior {

class Robot_looks_a_given_position : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    RhobanSSLAnnotation::Annotations annotations;
    ContinuousAngle angle;

    public:
    Robot_looks_a_given_position(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

	virtual Control control() const;

    void set_direction( const ContinuousAngle & angle );
    void set_direction( const Vector2d & vector );

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Robot_looks_a_given_position();
};

};
}; //Namespace Rhoban

#endif
