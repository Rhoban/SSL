#include "api.h"
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DEBUG(message) fprintf( stderr, "# %s -- %s:%d\n", message, __FILE__, __LINE__ ) 

void setBehaviour(
    struct Config * config, struct Robot * robots, int nb_robots, int team
){
    DEBUG( "Set Behaviour of ENSEIRB : " );
}

struct Robot * robots = 0;
int nb_robots = 0;

static int nb_of_valid_robots;
static int* valid_robots = 0; 
static int* id_robot_to_id_in_valid_robot_id = 0; 

void create_map_from_id_to_robot_id(int nb_robots){
    int max_id = -1;
    for( int i=0; i<nb_robots; i++ ){
        if( robots[i].team == ALLY && robots[i].id > max_id ){
            max_id = robots[i].id;
        }
    }
    id_robot_to_id_in_valid_robot_id = (int*) malloc( (max_id+1) * sizeof(int) );
    for( int i=0; i<max_id+1; i++ ){
        id_robot_to_id_in_valid_robot_id[i] = -1;
    }
}

void update_map_from_id_to_robot_id(){
    for( int i=0; i<nb_of_valid_robots; i++){
        id_robot_to_id_in_valid_robot_id[ valid_robots[i] ] = i;
    }
}

void free_map_from_id_to_robot_id(){
    free( id_robot_to_id_in_valid_robot_id );
    id_robot_to_id_in_valid_robot_id = 0;
}

void free_valid_robots_table(){
    free(valid_robots);
    valid_robots = 0;
}

void update_valid_robots_table(int nb_robots){
    nb_of_valid_robots  = 0;
    for( int i=0; i<nb_robots; i++ ){
        if( robots[i].is_valid && robots[i].team == ALLY){
            nb_of_valid_robots ++ ;
        }
    }
    free( valid_robots );
    valid_robots = (int*) malloc( nb_of_valid_robots * sizeof(int) );
    int j=0;
    for( int i=0; i<nb_robots; i++ ){
        if( robots[i].is_valid  && robots[i].team == ALLY ){
            valid_robots[j] = robots[i].id;
            j++;
        }
    }
}


void start_strategy(
    struct Config * config,
    struct Robot * _robots,
    int _nb_robots,
    int team
){
    DEBUG( "Start ENSEIRB strategy" );
    robots = _robots;
    nb_robots = _nb_robots;
    create_map_from_id_to_robot_id(nb_robots);
}

void stop_strategy(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team
){
    free_map_from_id_to_robot_id();
    free_valid_robots_table();
    DEBUG( "Stop ENSEIRB strategy" );
}

void update_strategy(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team,
    double time
){
    update_valid_robots_table(nb_robots);
    update_map_from_id_to_robot_id();
}

struct Action getBehaviour(
    struct Config * config, struct Robot * robots, int nb_robots, struct Ball * ball, int id_in_robots
){
    struct Robot* robot = robots + id_in_robots;
    int robot_id = robot->id;
    int i = id_robot_to_id_in_valid_robot_id[robot_id];

    struct Action result;
    result.dribler = 0;
    result.kicker = DO_NOTHING;
    result.x = -(i/2)*0.4;
    result.y = ((i%2==0) ? -1:1) * (1+i/2)*0.4;
    result.t = i*2*M_PI/nb_robots; 
    return result;
}
