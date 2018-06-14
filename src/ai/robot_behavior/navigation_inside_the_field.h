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

#ifndef __ROBOT_BEHAVIOR__NAVIGATION_INSIDE_THE_FIELD__H__
#define __ROBOT_BEHAVIOR__NAVIGATION_INSIDE_THE_FIELD__H__

#include "robot_behavior.h"
#include "navigation_with_obstacle_avoidance.h"
#include <AiData.h>

namespace RhobanSSL {
namespace Robot_behavior {

/*
 * This is an implementation of the article : 
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class Navigation_inside_the_field :
    public ConsignFollower 
{
    private:
        bool following_position_wad_updated;
        Navigation_with_obstacle_avoidance position_follower;

        Vector2d target_position;
        ContinuousAngle target_angle;
        rhoban_geometry::Point deviation_position;

        RhobanSSLAnnotation::Annotations annotations;

    public:
        Navigation_inside_the_field(
            Ai::AiData & ai_data, double time, double dt
        ); 

    protected:
        void update_control(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

    public:
        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );


        virtual Control control() const;

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit,
            double translation_acceleration_limit,
            double rotation_acceleration_limit
        );

        virtual void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );
        virtual void avoid_the_ball(bool value = true);

        virtual RhobanSSLAnnotation::Annotations get_annotations() const;
};

};
}; //Namespace Rhoban

#endif
