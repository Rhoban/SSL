//*****************************************************************************
//
// File Name	: 'constants.h'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen.000@gmail.com
// Created	: jeudi, juin 21 2018
// Revised	:
// Version	:
// Target MCU	:
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//
// Notes:	notes
//
//*****************************************************************************

#if !defined(CONSTANTS_H)
#define CONSTANTS_H

#include <math/frame_changement.h>
#include <math/position.h>

#include "team_color.h"


namespace RhobanSSL{
namespace Ai{
struct Constants {

  static constexpr int NB_OF_ROBOTS_BY_TEAM = 16;


  bool is_in_simulation;

  int frame_per_second;
  double period;

  double robot_radius;
  double radius_ball;
  Vector2d waiting_goal_position;
  int default_goalie_id;

// PID for translation
  double p_translation;
  double i_translation;
  double d_translation;
// PID for orientation
  double p_orientation;
  double i_orientation;
  double d_orientation;

  bool enable_kicking;

  double penalty_rayon;
  double translation_velocity_limit;
  double rotation_velocity_limit;
  double translation_acceleration_limit;
  double rotation_acceleration_limit;

  double time_limit_between_collision; 
  double security_acceleration_ratio;
  double obstacle_avoidance_ratio;

  double radius_security_for_collision;    
  double radius_security_for_avoidance;

  double wheel_radius;
  double wheel_excentricity;
  double wheel_nb_turns_acceleration_limit;

  double rules_avoidance_distance;
  double convergence_coefficient;
  double coefficient_to_increase_avoidance_convergence;

  void load( const std::string & config_path );

  Constants( const std::string & config_path, bool is_in_simulation );
};

}
}
#endif
