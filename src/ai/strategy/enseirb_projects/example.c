#include "api.h"
#include <stdio.h>
#include <math.h>
#define DEBUG(message) fprintf( stderr, "# %s -- %s:%d\n", message, __FILE__, __LINE__ ) 

void setBehaviour(
    struct Config * config, struct Robot * robots, int nb_robots, int team
){
    DEBUG( "Configuration de ENSEIRB : " );
    fprintf(
        stderr,
        "config -- width : %f, height : %f, goal_size : %f \n", 
        config->width, config->height, config->goal_size 
    );
}

struct Action getBehaviour(
    struct Config * config, struct Robot * robots, int nb_robots, struct Ball * ball, int robot_id
){
    struct Action result;
    result.id = DONT_HAVE_BALL;
    result.x = -robot_id*0.4;
    result.y = robot_id*0.4;
    result.t = robot_id*2*M_PI/nb_robots; 
    return result;
}
