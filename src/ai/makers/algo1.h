#ifndef __ALGO1__H_
#define __ALGO1__H_

#include "avoid_obstacle.h"

class Algo1 : Avoid_obstacle {
      public:
      double discretisation_size;
      

      Algo1( double discretisation_size);

      virtual void set_constraint(
            double width_field, double height_field,
            const rhoban_geometry::Point & starting_linear_position,
            const Vector2d & starting_linear_velocity,
            const rhoban_geometry::Point & ending_linear_position,
            const Vector2d & ending_linear_velocity,
            double robot_radius,
            const std::list< Obstacle > & obstacles, 
            double minimal_radius_curve
      );

      /*
          This function take a parameter u in [0, 1.0] and return the postion 
          of the robot.
      */
      virtual rhoban_geometry::Point linear_position( double u ) const;
};

#endif
