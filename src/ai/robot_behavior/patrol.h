/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#ifndef __ROBOT_BEHAVIOR__PATROL__H__
#define __ROBOT_BEHAVIOR__PATROL__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Patrol : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    int zone;
    std::vector< rhoban_geometry::Point > traject;

    public:
    Patrol(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

    static Patrol* two_way_trip( Ai::AiData& ai_data );
    static Patrol* tour_of_the_field( Ai::AiData& ai_data );

    void set_traject( const std::vector< rhoban_geometry::Point > & traject );

	virtual Control control() const;
    
    RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Patrol();

};

};
}; //Namespace Rhoban

#endif
