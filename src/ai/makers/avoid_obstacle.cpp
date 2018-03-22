#include "avoid_obstacle.h"

Obstacle::Obstacle( double radius, const rhoban_geometry::Point & linear_position ): 
    radius(radius),
    linear_position(linear_position)
{ }

Avoid_obstacle::~Avoid_obstacle(){
}

