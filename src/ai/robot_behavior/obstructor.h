/*
    This file is part of SSL.

    Copyright 2018 TO COMPLETE

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

#ifndef __ROBOT_BEHAVIOR__OBSTRUCTOR__H__
#define __ROBOT_BEHAVIOR__OBSTRUCTOR__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior
{

class Obstructor : public RobotBehavior
{
  private:
    rhoban_geometry::Point point_to_obstruct;
    int robot_to_obstruct_id;
    Vision::Team robot_to_obstruct_team;

    ConsignFollower *follower;

  public:
    Obstructor(Ai::AiData &ai_data);

    virtual void update(
        double time,
        const Ai::Robot &robot,
        const Ai::Ball &ball);

    virtual Control control() const;
    void declare_robot_to_obstruct( int robot_id, Vision::Team team = Vision::Team::Opponent );

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;
    virtual ~Obstructor();
};

}; // namespace Robot_behavior
}; // namespace RhobanSSL

#endif
