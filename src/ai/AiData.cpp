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
#include "AiData.h"
#include <assert.h>
#include <physic/factory.h>
#include <debug.h>
#include <physic/collision.h>
#include <physic/movement_on_new_frame.h>
#include <json/json.h>
#include <json/reader.h>
#include <fstream>

namespace RhobanSSL {
namespace Ai {

RobotPlacement::RobotPlacement():
  goal_is_placed(false)
{ };
RobotPlacement::RobotPlacement(
  std::vector< Position > field_robot_position,
  Position goalie_position
  ):
  goal_is_placed(true),
  field_robot_position(field_robot_position),
  goalie_position(goalie_position)
{ }
RobotPlacement::RobotPlacement(
  std::vector< Position > field_robot_position
  ):
  goal_is_placed(false),
  field_robot_position(field_robot_position)
{ }

Object::Object( const Object& object ):
  vision_data(object.vision_data), movement( object.movement->clone() )
{ }

Object& Object::operator=( const Object& object ){
  this->vision_data = vision_data;
  this->movement = object.movement->clone();
  return *this;
}


void Object::set_vision_data( const Vision::Object & vision_data  ){
  this->vision_data = vision_data;
  this->movement->set_sample( this->vision_data.movement );
}
void Object::set_movement( Movement * movement ){
  if( this->movement ){
    assert( movement != static_cast<Movement_on_new_frame*>(this->movement)->get_original_movement() );
    delete this->movement;
  }
  // We change the frame according referee informatiosns
  this->movement = new Movement_on_new_frame(movement);
}
void Object::change_frame(
  const rhoban_geometry::Point & origin,
  const Vector2d & v1, const Vector2d & v2
  ){
  static_cast<Movement_on_new_frame*>(movement)->set_frame(origin, v1, v2);
}

const RhobanSSL::Movement & Object::get_movement() const {
  return *movement;
}

Object::~Object(){
  delete movement;
}

Object::Object():
  movement(0)
{ }

bool Object::is_present_in_vision() const {
  return vision_data.isOk();
}



void AiData::update( const Vision::VisionData vision_data ){
  if( vision_data.field.present ){
    static_cast<Vision::Field&>(field) = vision_data.field;
  };

  for( auto team : {Vision::Ally, Vision::Opponent} ){
    for( int k=0; k<Vision::Robots; k++ ){
      robots[team][k].set_vision_data(
        vision_data.robots.at(team).at(k)
        );
    }
  }
  ball.set_vision_data( vision_data.ball );
  compute_table_of_collision_times();
}

void AiData::change_frame_for_all_objects(
  const rhoban_geometry::Point & origin,
  const Vector2d & v1, const Vector2d & v2
  ){
  team_point_of_view.set_frame( origin, v1, v2 );
  for( auto team : {Vision::Ally, Vision::Opponent} ){
    for( int k=0; k<Vision::Robots; k++ ){
      robots[team][k].change_frame( origin, v1, v2 );
    }
  }
  ball.change_frame( origin, v1, v2 );
};

void AiData::change_team_color( Ai::Team team_color ){
  this->team_color = team_color;
}

AiData::AiData( Data& data_for_thread, const std::string & config_path, bool is_in_simulation, Ai::Team team_color ):
  team_color(team_color),
  constants(config_path, is_in_simulation),
  dt(constants.period),
  data_for_thread(data_for_thread)
{
  int nb_robots = 0;
  for( auto team : {Vision::Ally, Vision::Opponent} ){
    for( int k=0; k<Vision::Robots; k++ ){
      robots[team][k].set_movement(
        physic::Factory::robot_movement(*this)
        );
      nb_robots++;
    }
  }
  all_robots = std::vector< std::pair<Vision::Team, Robot*> >(nb_robots);
  unsigned int i = 0;
  for( auto team : {Vision::Ally, Vision::Opponent} ){
    for( int k=0; k<Vision::Robots; k++ ){
      all_robots[i] = std::pair< Vision::Team, Robot* >(
        team, &( robots.at(team).at(k) )
        );
      i++;
    }
  }
  ball.set_movement(
    physic::Factory::ball_movement(*this)
    );

}


bool AiData::robot_is_inside_the_field( int robot_id ) const {
  const RhobanSSL::Movement & mov = robots.at(Vision::Team::Ally).at(robot_id).get_movement();
  return field.is_inside( mov.linear_position(time) );
}

bool AiData::robot_is_valid( int robot_id ) const {
  return (
    robots.at(Vision::Team::Ally).at(robot_id).is_present_in_vision()
    and
    robot_is_inside_the_field(robot_id)
    );
}

const AiData::Collision_times_table & AiData::get_table_of_collision_times() const
{
  return table_of_collision_times;
}

void AiData::visit_all_pair_of_robots(
  std::function <
  void (
    Vision::Team robot_team_1, Robot & robot_1,
    Vision::Team robot_team_2, Robot & robot_2
    )
  > visitor
  ){
  for( unsigned int i = 0; i < all_robots.size(); i++ ){
    for( unsigned int j = i+1; j < all_robots.size(); j++ ){
      visitor(
        all_robots[i].first, *all_robots[i].second,
        all_robots[j].first, *all_robots[j].second
        );
    }
  }
}

std::list< std::pair<int, double> > AiData::get_collisions(
  int robot_id, const Vector2d & velocity_translation
  ) const {
  std::list< std::pair< int, double> > result;
  const Robot * robot_1 = &( robots.at(Vision::Team::Ally).at(robot_id) );

  if( not( robot_1->is_present_in_vision() ) ){
    return {};
  }

  for( unsigned int i=0; i<all_robots.size(); i++ ){
    const Robot * robot_2 = all_robots[i].second;
    if( not( robot_2->is_present_in_vision() ) ){
      continue;
    }
    if( robot_1->id() != robot_2->id() or all_robots[i].first != Vision::Team::Ally ){
      double radius_error = constants.radius_security_for_collision;
      std::pair<bool, double> collision = collision_time(
        constants.robot_radius,
        robot_1->get_movement().linear_position( robot_1->get_movement().last_time() ),
        velocity_translation,
        constants.robot_radius,
        robot_2->get_movement().linear_position( robot_2->get_movement().last_time() ),
        robot_2->get_movement().linear_velocity( robot_2->get_movement().last_time() ),
        radius_error
        );
      if( collision.first ){
        result.push_back( std::pair<int,double>( i, collision.second ) );
      }
    }
  }
  return result;
}

void AiData::compute_table_of_collision_times(){
  table_of_collision_times.clear();
  for( unsigned int i=0; i<all_robots.size(); i++ ){
    for( unsigned int j=i+1; j<all_robots.size(); j++ ){
      Robot & robot_1 = *all_robots[i].second;
      Robot & robot_2 = *all_robots[j].second;
      double radius_error = constants.radius_security_for_collision;
      std::pair<bool, double> collision = collision_time(
        constants.robot_radius,
        robot_1.get_movement(),
        constants.robot_radius,
        robot_2.get_movement(),
        radius_error, time
        );
      if( collision.first ){
        table_of_collision_times[ std::pair<int,int>(i,j) ] = collision.second;
      }
    }
  }
}

rhoban_geometry::Point AiData::relative2absolute(
  double x, double y
  ) const {
  return rhoban_geometry::Point(
    field.fieldLength/2.0*x, field.fieldWidth/2.0*y
    );
}
rhoban_geometry::Point AiData::relative2absolute(
  const rhoban_geometry::Point & point
  ) const {
  return relative2absolute( point.getX(), point.getY() );
}


RobotPlacement AiData::default_attacking_kickoff_placement() const {
  //
  //     A.
  // B        C
  //      D
  //   E     F
  //      G
  //
  return RobotPlacement(
    {
      Position( -0.8, .0, 0.0 ), // A
        Position( relative2absolute(-1.0/3.0, 2.0/3.0), 0.0), // B
        Position( relative2absolute(-1.0/3.0, -2.0/3.0), 0.0), // C
        Position( relative2absolute(-2.0/3.0, 1.0/2.0), 0.0), // D
        Position( relative2absolute(-2.0/3.0, -1.0/2.0), 0.0), // E
        Position( relative2absolute(-1.5/3.0, 0.0), 0.0), // F
        Position( relative2absolute(-2.5/3.0, 0.0), 0.0), // G
        },
    Position(
      relative2absolute(-1.0, 0.0), 0.0  // G
      )
    );
}

RobotPlacement AiData::default_defending_kickoff_placement() const {
  //
  //      .
  // B    A    C
  //      F
  //   D     E
  //      G
  //
  return RobotPlacement(
    {
      Position( relative2absolute(-1.0/3.0, 0.0), 0.0 ), // A
        Position( relative2absolute(-1.0/3.0, 2.0/3.0), 0.0), // B
        Position( relative2absolute(-1.0/3.0, -2.0/3.0), 0.0), // C
        Position( relative2absolute(-2.0/3.0, 1.0/2.0), 0.0), // D
        Position( relative2absolute(-2.0/3.0, -1.0/2.0), 0.0), // E
        Position( relative2absolute(-1.5/3.0, 0.0), 0.0), // F
        Position( relative2absolute(-2.5/3.0, 0.0), 0.0), // G
        },
    Position(
      relative2absolute(-1.0, 0.0), 0.0  // G
      )
    );
}

std::ostream& operator<<(std::ostream& out, const Object& object){
  out << "visible : " << object.is_present_in_vision() << ", vision : " << object.vision_data;
  return out;
}

    
Robot::Robot():
  is_goalie(false),
  infra_red(false)
{}


} } //Namespace
