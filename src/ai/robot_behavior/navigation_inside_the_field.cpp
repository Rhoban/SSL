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

#include <debug.h>
#include <core/print_collection.h>
#include <core/collection.h>


namespace RhobanSSL {
namespace Robot_behavior {

Navigation_inside_the_field::Navigation_inside_the_field(
    Ai::AiData & ai_data, double time, double dt
):
    ConsignFollower(ai_data),
    need_to_avoid_the_ball(true),
    saving_ball_radius_avoidance(ai_data.constants.robot_radius),
    following_position_was_updated(true),
    position_follower(ai_data, time, dt),
    target_position(0.0, 0.0), target_angle( 0.0),
    deviation_position(0.0, 0.0)
{
}

void Navigation_inside_the_field::set_following_position(
    const Vector2d & position_to_follow,
    const ContinuousAngle & target_angle
){
    following_position_was_updated = true;
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
    if( ai_data.force_ball_avoidance ){
        this->position_follower.set_radius_avoidance_for_the_ball(
            get_robot_radius() + get_ball_radius() + ai_data.constants.rules_avoidance_distance
        );
        this->avoid_the_ball( true );
    }else{
        this->position_follower.set_radius_avoidance_for_the_ball(
            saving_ball_radius_avoidance   
        );
        this->avoid_the_ball( need_to_avoid_the_ball );
    }

    double marge = 2*get_robot_radius();
    if( following_position_was_updated ){
        // Box cropped_field(
        //     field_SW() + Vector2d( marge, marge ),
        //     field_NE() - Vector2d( marge, marge)
        // );
        // Box cropped_field(
        //     field_SW(),
        //     field_NE()
        // );
        // Trying agressive margins
        Box cropped_field(
            field_SW() - Vector2d( marge, marge),
            field_NE() + Vector2d( marge, marge)
        );
        float radius_margin_factor=2.0;
        Box opponent_penalty = opponent_penalty_area().increase(get_robot_radius());
        Box ally_penalty = ally_penalty_area().increase(get_robot_radius());

        Box opponent_penalty_large = opponent_penalty_area().increase(get_robot_radius()*radius_margin_factor);
        Box ally_penalty_large = ally_penalty_area().increase(get_robot_radius()*radius_margin_factor);

        rhoban_geometry::Point robot_position = linear_position();
        double error = get_robot_radius()*radius_margin_factor;

        if( opponent_penalty.is_inside(robot_position) ){
            // If we're in their penalty
            deviation_position = rhoban_geometry::Point( opponent_penalty.get_SW().getX() - error, robot_position.getY() );
        }else if( not(is_goalie()) and ally_penalty.is_inside(robot_position) ){
            // If we're in our penalty
            deviation_position = rhoban_geometry::Point( ally_penalty.get_NE().getX() + error, robot_position.getY() );
        }else{
            if( cropped_field.is_inside( vector2point(target_position) ) ){
                // Normal case, the goal position is in the field
                deviation_position = vector2point( target_position );
            }else{
                // Changing the target position to match the closest segment
                cropped_field.closest_segment_intersection(
                    robot_position, vector2point( target_position ),
                    deviation_position
                );
            }
            //Here, deviation_position should be inside the field, but it could still be in a penalty area.
            if( not( is_goalie() ) and ally_penalty.is_inside( deviation_position ) ){
                ally_penalty_large.closest_segment_intersection(
                    robot_position, vector2point( deviation_position ),
                    deviation_position
                );
                if( not(cropped_field.is_inside( deviation_position)) ){
                    deviation_position = deviation_position + Vector2d( penalty_area_depth() + error, 0.0 );
                }
            }else if( opponent_penalty.is_inside( deviation_position ) ){
                opponent_penalty_large.closest_segment_intersection(
                    robot_position, vector2point( deviation_position ),
                    deviation_position
                );
                if( not(cropped_field.is_inside( deviation_position)) ){
                    deviation_position = deviation_position - Vector2d( penalty_area_depth() + error, 0.0 );
                }
            }
        }

        if ((not( is_goalie() ) && ally_penalty.is_inside( deviation_position )) || opponent_penalty.is_inside( deviation_position )) {
            DEBUG("Damn, deviation_position is still inside a penalty (this is a bug)...");
        }

        this->position_follower.set_following_position( deviation_position, target_angle );
        following_position_was_updated = false;
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
    need_to_avoid_the_ball = value;
    position_follower.avoid_the_ball(value);
}

void Navigation_inside_the_field::avoid_ally(bool value){
    position_follower.avoid_ally(value);
}

void Navigation_inside_the_field::avoid_opponent(bool value){
    position_follower.avoid_opponent(value);
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

void Navigation_inside_the_field::set_radius_avoidance_for_the_ball(
    double radius
){
    position_follower.set_radius_avoidance_for_the_ball(radius);
    saving_ball_radius_avoidance = radius;
}


}
}
