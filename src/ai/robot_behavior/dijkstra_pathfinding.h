/*
    This file is part of SSL.

    Copyright 2018 Rémi Fabre (remifabre1800@gmail.com)

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

#ifndef __ROBOT_BEHAVIOR__A_STAR_DIJKSTRA_PATHFINDING__H__
#define __ROBOT_BEHAVIOR__A_STAR_DIJKSTRA_PATHFINDING__H__

#include "robot_behavior.h"
#include "consign_follower.h"
#include <AiData.h>

namespace RhobanSSL {
namespace Robot_behavior {

/*
 * This is an implementation of the article : 
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class Dijkstra_pathfinding : public ConsignFollower  {
    private:
        ConsignFollower* follower;

        Vector2d target_position;
        ContinuousAngle target_angle;

    public:
        Dijkstra_pathfinding(
            Ai::AiData & ai_data, double time, double dt,
            ConsignFollower* consign_follower
        ); 

    public:
        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

	void compute_next_position();

        virtual Control control() const;

        virtual void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

	virtual ~Dijkstra_pathfinding();
};

};
}; //Namespace Rhoban

#endif
