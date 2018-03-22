#include "algo1.h"

Algo1::Algo1( double discretisation_size ):
   discretisation_size( discretisation_size )
{
}

void Algo1::set_constraint(
    double width_field, double height_field,
    const rhoban_geometry::Point & starting_linear_position,
    const Vector2d & starting_linear_velocity,
    const rhoban_geometry::Point & ending_linear_position,
    const Vector2d & ending_linear_velocity,
    double robot_radius,
    const std::list< Obstacle > & obstacles, 
    double minimal_radius_curve
){
}

/*
  This function take a parameter u in [0, 1.0] and return the postion 
  of the robot.
*/
rhoban_geometry::Point Algo1::linear_position( double u ) const {
	return rhoban_geometry::Point( 0.0, 0.0 );
}

