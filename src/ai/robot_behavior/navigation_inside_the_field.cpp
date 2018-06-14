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

#include "navigation_inside_the_field.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>
#include <physic/collision.h>
#include <math/box.h>
#include <math/intersection.h>

namespace RhobanSSL {
namespace Robot_behavior {

Navigation_inside_the_field::Navigation_inside_the_field(
    Ai::AiData & ai_data, double time, double dt
):
    ConsignFollower(ai_data), 
    following_position_wad_updated(true),
    position_follower(ai_data, time, dt),
    target_position(0.0, 0.0), target_angle( 0.0),
    deviation_position(0.0, 0.0)
{
} 

void Navigation_inside_the_field::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & target_angle
){
    following_position_wad_updated = true;
    this->target_position = position_to_follow;
    this->target_angle = target_angle;
}


void Navigation_inside_the_field::update(
    double time,
    const Ai::Robot & robot,
    const Ai::Ball & ball
){
    // At First, we update time and update potition from the abstract class robot_behavior.
    // DO NOT REMOVE THAT LINE
    RobotBehavior::update_time_and_position( time, robot, ball );

    update_control( time, robot, ball );
}

void Navigation_inside_the_field::update_control(
    double time, const Ai::Robot & robot, const Ai::Ball & ball
){
    if( following_position_wad_updated ){
        Box cropped_field(
            field_SW() + Vector2d( get_robot_radius(), get_robot_radius() ),
            field_NE() - Vector2d( get_robot_radius(), get_robot_radius() )
        );

        if( cropped_field.is_inside( vector2point( target_position ) ) ){
            this->deviation_position = vector2point( target_position );
        }else{
            cropped_field.closest_segment_intersection(
                linear_position(), vector2point( target_position ),
                this->deviation_position
            );
        }

#if  0 // TODO       
        Box opponent_penalty = opponent_penalty_area().increase(get_robot_radius());
        if( opponent_penalty.is_inside( vector2point( this->deviation_position ) ) ){
            opponent_penalty.closest_segment_intersection(
                linear_position(), this->deviation_position,
                this->deviation_position
            );
        }

        Box ally_penalty = ally_penalty_area().increase(get_robot_radius());
        if( ally_penalty.is_inside( vector2point( this->deviation_position ) ) ){
            ally_penalty.closest_segment_intersection(
                linear_position(), this->deviation_position,
                this->deviation_position
            );
        }
#endif
        this->position_follower.set_following_position( this->deviation_position, target_angle );
        following_position_wad_updated = false;
    }
    position_follower.update( time, robot, ball );
}

Control Navigation_inside_the_field::control() const {
    return position_follower.control();
}

void Navigation_inside_the_field::set_translation_pid( double kp, double ki, double kd ){
    position_follower.set_translation_pid( kp, ki, kd );
}
void Navigation_inside_the_field::set_orientation_pid( double kp, double ki, double kd ){
    position_follower.set_orientation_pid( kp, ki, kd );
}

void Navigation_inside_the_field::avoid_the_ball(bool value){
    position_follower.avoid_the_ball(value);
}

void Navigation_inside_the_field::set_limits(
    double translation_velocity_limit,
    double rotation_velocity_limit,
    double translation_acceleration_limit,
    double rotation_acceleration_limit
){
    position_follower.set_limits(
        translation_velocity_limit,
        rotation_velocity_limit,
        translation_acceleration_limit,
        rotation_acceleration_limit
    );
}

RhobanSSLAnnotation::Annotations Navigation_inside_the_field::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    if( norm_square(deviation_position-target_position) > 0.001 ){
        annotations.addArrow( deviation_position, target_position, "yellow" );
    }
    annotations.addAnnotations( position_follower.get_annotations() );
    //annotations.addBox( opponent_penalty_area(), "red" );
    //annotations.addBox( ally_penalty_area(), "red" );
    return annotations;
}


}
}
