#ifndef __AVOID_OBSTACLE__H__
#define __AVOID_OBSTACLE__H__

#include <rhoban_geometry/point.h>
#include <math/vector.h>
#include <list>
#include "Obstacle.h"

class Avoid_obstacle {
   public:

      /* 
         This function is used to configurate the datas of the obstacle and of some constraint
         on the robot in order to compute a move for the robot.
       */
      virtual void set_constraint(
            double width_field, double height_field,
            const rhoban_geometry::Point & starting_linear_position,
            const Vector2d & starting_linear_velocity,
            const rhoban_geometry::Point & ending_linear_position,
            const Vector2d & ending_linear_velocity,
            double robot_radius,
            const std::list< Obstacle > & obstacles, 
            double minimal_radius_curve
      ) = 0;

      /*
          This function take a parameter u in [0, 1.0] and return the postion 
          of the robot.
      */
      virtual rhoban_geometry::Point linear_position( double u ) const = 0;

      virtual ~Avoid_obstacle();
};

#endif
