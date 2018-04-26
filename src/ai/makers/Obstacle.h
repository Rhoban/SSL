#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

typedef struct Obstacle Obstacle;

struct Obstacle {
    double radius;
    rhoban_geometry::Point linear_position;
	
    Obstacle( double radius, const rhoban_geometry::Point & linear_position );
};

#endif /* __OBSTACLE_H__ */
