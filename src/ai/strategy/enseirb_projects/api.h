#ifndef __STRATEGY__ENSEIRB__API_H__
#define __STRATEGY__ENSEIRB__API_H__

#ifdef __cplusplus
namespace enseirb {
extern "C"
{
#endif

//
// This Config structure define 
// the field geometry and some 
// extra information of the robocup.
//
struct Config {
    double width, height;
    double margin;
    double goal_size;
};

typedef enum {
    ALLY,
    OPPONENT
} Team;

struct Robot {
    int id; // the identifer of the robot
    double radius; // the radius of the robot
    double x, y, t; //linear position (x,y) and  angular position (t)
    double vx, vy, vt; // linear velocity (vx, vy) and angular velocity (vt)
    Team team;  // the team of the robot 
    int is_goal; // True if it is the goal
    int is_valid; // Set to true if the robot is in game and in a valid state
};

struct Ball {
    double radius;  // The ball radius
    double x, y, vx, vy; // linear position x,y and linear velocity vx, vy
};

typedef enum {
    DO_NOTHING,
    SHOOT,
    LOBB
} Kicker;

struct Action {
    int dribler; // a boolean : true if dribler should be actived 
    Kicker kicker; // Define if kicker should be actived
    double x, y, t;// linear position x,y and angular position t
};


// This function is called when we start a strategy.
// In a match a strategy is called several times.
// When a strategy is started, the array robots, is 
// not rallocated. The robot table is free just
// when the strategy is finished (stoped). 
void start_strategy(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team
);

// This function is call at each iteration of the 
// main program
void update_strategy(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team,
    double time   // the time of the clock 
);

// This function is called when we stop (finish) a strategy.
// When a strategy is finished, the robot array can be reallocated.
void stop_strategy(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team
);

void setBehaviour(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    int team
);

//
// At each loop iteration , for all robots of your team that is present 
// in the robots table, getBeahviour is called.
//
// All the getBehaviour function are called after update_strategy()
//
struct Action getBehaviour(
    struct Config * config,
    struct Robot * robots,
    int nb_robots,
    struct Ball * ball,
    int robot_id
);

#ifdef __cplusplus
}
}
#endif

#endif
