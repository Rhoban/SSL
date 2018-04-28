/*! \mainpage Index

 * \section intro_sec Introduction
 *
 * This project aims to create a behavior simulator for the Small Size League Robot,
 * along with a responsive visual simulator.
 *
 * \section install_sec Install
 *
 * In order to compile the program from source, you must install all the following
 * SDL dependencies along with their corresponding development package : libsdl2-2.0-0, libsdl2-ttf-2.0-0,
 * libsdl2-gfx-1.0-0, libsdl2-image-2.0-0.
 *
 * Then, move into the "build" repository and run "cmake ..".
 * Finally, enter the command "make all" to build the main execuatble along with the tests.
 *
 *
 *
 *
 */

#ifndef __STRATEGY__ENSEIRB__API_H__
#define __STRATEGY__ENSEIRB__API_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Structures

struct Config {
    double width, height;
    double goal_size;
};

#define GOAL_KEEPER 0
#define DEFENDER  1
#define STRIKER 2

#define TEAM_1 0
#define TEAM_2 1

struct Behaviour {
    int id;
    void * data;
};

#define MARKING 0
#define FIRST_DEFENDER_IN_COORDINATED_DEFENSE_STRATEGY 1
#define SECOND_DEFENDER_IN_COORDINATED_DEFENSE_STRATEGY 2

struct DefenderBehaviourData {
    int id;
    int target_id;
};

struct Robot {
    int id;
    double radius;
    double x, y, t;
    double vx, vy, vt;
    int team;
    struct Behaviour behaviour;
};

struct Ball {
    double radius;
    double x, y, vx, vy;
};

#define DONT_HAVE_BALL 0
#define DRIBBLE 1
#define SHOOT 2
#define LOBB 3

struct Action {
    int id;
    double x, y, t;
};

// Fonctions

void setBehaviour(struct Config * config,
                  struct Robot * robots,
                  int nb_robots,
                  int team);

struct Action getBehaviour(struct Config * config,
                           struct Robot * robots,
                           int nb_robots,
                           struct Ball * ball,
                           int robot_id);

// Autres constantes

#define DEFENDER_GOAL_KEEPER_DISTANCE_IN_COORDINATED_DEFENSE_STRATEGY 2

#ifdef __cplusplus
}
#endif


#endif
