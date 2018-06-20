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

#include "navigation_with_obstacle_avoidance.h"
#include <rhoban_geometry/segment.h>
#include <physic/constants.h>
#include <physic/collision.h>

namespace RhobanSSL {
namespace Robot_behavior {

Navigation_with_obstacle_avoidance::Navigation_with_obstacle_avoidance(
  Ai::AiData & ai_data, double time, double dt
  ):
  ConsignFollower(ai_data), 
  ignore_the_ball(false),
  ignore_ally(false),
  ignore_opponent(false),
  ball_radius_avoidance( ai_data.constants.robot_radius ),
  position_follower(ai_data, time, dt),    position_follower_avoidance(ai_data, time, dt),
  target_position(0.0, 0.0), target_angle(0.0)
{
} 

void Navigation_with_obstacle_avoidance::set_following_position(
  const Vector2d & position_to_follow,
  const ContinuousAngle & target_angle
  ){
  this->position_follower.set_following_position( position_to_follow, target_angle ); 
  this->target_position = position_to_follow;
  this->target_angle = target_angle;
  this->target_angle = this->robot_angular_position;
  this->target_angle.set_to_nearest(target_angle); 
}

void Navigation_with_obstacle_avoidance::determine_the_closest_obstacle(){
  // We determine the closest obstaclte in term of time collision.

  Control ctrl = position_follower.control();

  min_time_collision = -1;
  closest_robot = -1;
  std::list< std::pair<int, double> > collisions_with_ctrl = ai_data.get_collisions(
    robot().id(), ctrl.velocity_translation
    );
  assert(
    ai_data.constants.security_acceleration_ratio > ai_data.constants.obstacle_avoidance_ratio
    );
  double ctrl_velocity_norm = ctrl.velocity_translation.norm();
  double time_to_stop = ctrl_velocity_norm/(
    ai_data.constants.obstacle_avoidance_ratio
    *
    ai_data.constants.translation_acceleration_limit
    );

  for( const std::pair<int, double> & collision : collisions_with_ctrl ){
    if( ignore_ally and collision.first < Ai::Constants::NB_OF_ROBOTS_BY_TEAM){
      continue;
    }
    if( ignore_opponent and collision.first >= Ai::Constants::NB_OF_ROBOTS_BY_TEAM){
      continue;
    }
    double time_before_collision = collision.second;
    if( time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY ){
      if(
        ( min_time_collision == -1 ) 
        or
        ( min_time_collision > time_before_collision ) 
        ){
        min_time_collision = time_before_collision;
        closest_robot = collision.first;
      }
    }
  }
  ball_is_the_obstacle = false;
  if( not(ignore_the_ball) ){      
    double radius_error = ai_data.constants.radius_security_for_collision;
    std::pair<bool, double> collision = collision_time(
      ai_data.constants.robot_radius, 
      robot().get_movement().linear_position( robot().get_movement().last_time() ),
      ctrl.velocity_translation,
      ball_radius_avoidance, 
      ball().get_movement().linear_position( ball().get_movement().last_time() ),
      ball().get_movement().linear_velocity( ball().get_movement().last_time() ),
      radius_error
      );
    if( collision.first ){
      double time_before_collision = collision.second;
      if( time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY ){
        if(
          ( min_time_collision == -1 ) 
          or
          ( min_time_collision > time_before_collision ) 
          ){
          min_time_collision = time_before_collision;
          ball_is_the_obstacle = true;
        }
      }
    }
  }
}


void Navigation_with_obstacle_avoidance::compute_the_radius_of_limit_cycle(){
  //Is yet constructed at construction
  assert(
    ai_data.constants.radius_security_for_collision 
    < 
    ai_data.constants.radius_security_for_avoidance
    );

#if 1
  if ( ball_is_the_obstacle ) {
    assert( not(ignore_the_ball) ); // Normally determine_the_closest_obstacle() set ball_is_the_obstacle to false when we ignore the ball
    radius_of_limit_cycle = ai_data.constants.radius_ball + ai_data.constants.robot_radius + ai_data.constants.radius_security_for_avoidance;
  } else {     
    if( robot().get_movement().linear_velocity(ai_data.time).norm() < ai_data.constants.translation_velocity_limit/4.0 ){
      radius_of_limit_cycle = 2*ai_data.constants.robot_radius;// + ai_data.constants.radius_security_for_avoidance;   
    }else{
      radius_of_limit_cycle = 2*ai_data.constants.robot_radius + ai_data.constants.radius_security_for_avoidance;   
    }
  }
#endif
    

}

void Navigation_with_obstacle_avoidance::convert_cycle_direction_to_linear_and_angular_velocity(){
  // avoidance_control.kick = false;
  // avoidance_control.ignore = false;

    
  // Control follower_control = position_follower.control();
  // avoidance_control.velocity_rotation = follower_control.velocity_rotation;
  // avoidance_control.velocity_translation = limit_cycle_direction*(
  //     follower_control.velocity_translation.norm()/limit_cycle_direction.norm()
  // );

  assert(limit_cycle_direction.norm()!=0.0);

  rhoban_geometry::Point pos=linear_position() +  limit_cycle_direction/(limit_cycle_direction.norm())*1.0;

  position_follower_avoidance.set_following_position(pos , target_angle);
}

void Navigation_with_obstacle_avoidance::compute_the_limit_cycle_direction_for_obstacle(
  const rhoban_geometry::Point & obstacle_linear_position,
  const Vector2d & obstacle_linear_velocity
  ){
  obstacle_point_of_view.robot_linear_position = 
    vector2point( robot_linear_position - Vector2d( obstacle_linear_position ) )
    ;
  obstacle_point_of_view.target_linear_position =
    vector2point( target_position - Vector2d( obstacle_linear_position ) )
    ;
   
  /////////////////////////////////////////////////////////////////
  // We compute the sens of avoidance rotatiion
  /////////////////////////////////////////////////////////////////
  // sign_of_avoidance_rotation = 1.0; // TODO

  Ai::Robot & obstacle = *( ai_data.all_robots[closest_robot].second );
  Vector2d obstacle_to_goal=vector2point(target_position)-obstacle_linear_position;
  Vector2d current_to_goal=vector2point(target_position)-linear_position();
  double angle=vector2angle(current_to_goal).value()-vector2angle(obstacle_to_goal).value();

#if 0    
  if(angle<0.0)
  {
    sign_of_avoidance_rotation=1;
  }
  else{
    sign_of_avoidance_rotation=-1;
  }
#else
  sign_of_avoidance_rotation = 1.0; // TODO
#endif    
    
/////////////////////////////////////////////////////////////////
// We compute now the limit cycle rotation
/////////////////////////////////////////////////////////////////
  const rhoban_geometry::Point & s = obstacle_point_of_view.robot_linear_position;

  double XX=s.getX()*s.getX();
  double YY=s.getY()*s.getY();
  double delta_radius;

  if( (XX+YY)==0.0 )
    delta_radius=0.5; 
  else
    delta_radius = ( radius_of_limit_cycle*radius_of_limit_cycle - XX - YY )/(XX+YY)*ai_data.constants.coefficient_to_increase_avoidance_convergence;
  obstacle_point_of_view.limit_cycle_direction = Vector2d(
    sign_of_avoidance_rotation * s.getY() + s.getX() * delta_radius,
    - sign_of_avoidance_rotation * s.getX() + s.getY() * delta_radius
    );

  limit_cycle_direction = obstacle_point_of_view.limit_cycle_direction + obstacle_linear_velocity;
}

void Navigation_with_obstacle_avoidance::compute_the_limit_cycle_direction_for_robot(){
  /////////////////////////////////////////////////////////////////
  //We change the frame from absolute to frame relative to obstacle
  /////////////////////////////////////////////////////////////////
  Ai::Robot & obstacle = *( ai_data.all_robots[closest_robot].second );
  rhoban_geometry::Point obstacle_linear_position = obstacle.get_movement().linear_position( time() ); 
  Vector2d obstacle_linear_velocity = obstacle.get_movement().linear_velocity( time() ); 

  compute_the_limit_cycle_direction_for_obstacle( obstacle_linear_position, obstacle_linear_velocity );
}

void Navigation_with_obstacle_avoidance::compute_the_limit_cycle_direction_for_ball(){
  /////////////////////////////////////////////////////////////////
  //We change the frame from absolute to frame relative to obstacle
  /////////////////////////////////////////////////////////////////
  rhoban_geometry::Point obstacle_linear_position = ball().get_movement().linear_position( time() ); 
  Vector2d obstacle_linear_velocity = ball().get_movement().linear_velocity( time() ); 

  compute_the_limit_cycle_direction_for_obstacle( obstacle_linear_position, obstacle_linear_velocity );
}

void Navigation_with_obstacle_avoidance::compute_the_limit_cycle_direction(){
  if( ball_is_the_obstacle ){
    assert( not(ignore_the_ball) ); // Normally determine_the_closest_obstacle() set ball_is_the_obstacle to false when we ignore the ball
    compute_the_limit_cycle_direction_for_ball();
  }else{
    compute_the_limit_cycle_direction_for_robot();
  }
}

void Navigation_with_obstacle_avoidance::update(
  double time,
  const Ai::Robot & robot,
  const Ai::Ball & ball
  ){
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::update_time_and_position( time, robot, ball );
  // Now 
  //  this->robot_linear_position
  //  this->ball_position
  //  this->robot_angular_position 
  //  this->robot()
  // are all avalaible

  update_control( time, robot, ball );
}

void Navigation_with_obstacle_avoidance::update_control(
  double time, const Ai::Robot & robot, const Ai::Ball & ball
  ){

  position_follower.update( time, robot, ball ); // We use the future command to predict collision
  determine_the_closest_obstacle();
  if( min_time_collision >= 0 ){
    compute_the_radius_of_limit_cycle();
    compute_the_limit_cycle_direction();
    convert_cycle_direction_to_linear_and_angular_velocity();

    position_follower_avoidance.update( time, robot, ball );
  }
}

Control Navigation_with_obstacle_avoidance::control() const {
  if( min_time_collision >= 0 ){
    return position_follower_avoidance.control();
  }else{
    return position_follower.control();
  }
    
}

void Navigation_with_obstacle_avoidance::set_translation_pid( double kp, double ki, double kd ){
  position_follower.set_translation_pid( kp, ki, kd );
}
void Navigation_with_obstacle_avoidance::set_orientation_pid( double kp, double ki, double kd ){
  position_follower.set_orientation_pid( kp, ki, kd );
}

void Navigation_with_obstacle_avoidance::avoid_the_ball(bool value){
  ignore_the_ball = not(value);
}
void Navigation_with_obstacle_avoidance::avoid_ally(bool value){
  ignore_ally = not(value);
}
void Navigation_with_obstacle_avoidance::avoid_opponent(bool value){
  ignore_opponent = not(value);
}

void Navigation_with_obstacle_avoidance::set_limits(
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

RhobanSSLAnnotation::Annotations Navigation_with_obstacle_avoidance::get_annotations() const {
  RhobanSSLAnnotation::Annotations annotations;
//    annotations.addCircle( linear_position(), radius_of_limit_cycle );

  if( min_time_collision >= 0 ){
    annotations.addArrow(linear_position(),linear_position()+limit_cycle_direction*(limit_cycle_direction.norm())*10,"red");      
    if(closest_robot==-1){
      annotations.addCircle(ball().get_movement().linear_position(ai_data.time),radius_of_limit_cycle);
    } else {
      annotations.addCircle(ai_data.all_robots.at(closest_robot).second->get_movement().linear_position(ai_data.time),radius_of_limit_cycle);
    }
    annotations.addAnnotations(position_follower_avoidance.get_annotations());
  }else{
    annotations.clear();
    annotations.addAnnotations(position_follower.get_annotations());
  }

  return annotations;
}

void Navigation_with_obstacle_avoidance::set_radius_avoidance_for_the_ball(
  double radius
  ){
  ball_radius_avoidance = radius;
}

double Navigation_with_obstacle_avoidance::get_radius_avoidance_for_the_ball(){
  return ball_radius_avoidance;
}

}
}
