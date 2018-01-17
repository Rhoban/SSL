#include "AI.h"
#include <timing/TimeStamp.hpp>
#include <cmath>
#include <unistd.h>


// Comment the following line if you are working with the real robot.
// If you are working with the grSim simulator, don't comment.
#define SSL_SIMU

using namespace Utils::Timing;

namespace RhobanSSL
{

void limits_velocity( Control & ctrl ){
    if( ctrl.velocity_translation.norm() > TRANSLATION_VELOCITY_LIMIT ){
        ctrl.velocity_translation = Eigen::Vector2d(0.0, 0.0);
        std::cerr << "WARNING : we reached the "
            "limit translation velocity !" << std::endl;
    }

    if( std::abs( ctrl.velocity_rotation ) > ROTATION_VELOCITY_LIMIT ){
        ctrl.velocity_rotation = 0.0;
        std::cerr << "WARNING : we reached the "
            "limit rotation velocity !" << std::endl;
    }
}

void AI::prepare_to_send_control( int robot_id, Control ctrl ){
    limits_velocity(ctrl);

    #ifdef SSL_SIMU
        double sign_y = 1.0;
    #else
        double sign_y = -1.0;
    #endif

    #ifdef SSL_SIMU
    #else
    if( ctrl.kick ){
        commander->kick();
    }
    #endif

    commander->set(
        robot_id, true, 
        ctrl.velocity_translation[0], sign_y*ctrl.velocity_translation[1], 
        ctrl.velocity_rotation
    );
}

Control AI::update_goalie(
    double time, GameState::Robot & robot, GameState::Ball & ball
){
    if (robot.isOk()) {
        Eigen::Vector2d robot_position( 
            robot.position.getX(), robot.position.getY()
        );
        Eigen::Vector2d ball_position(
            ball.position.getX(), ball.position.getY()
        );
        double robot_orientation = robot.orientation.getSignedValue();

        goalie.update(
            ball_position, robot_position, robot_orientation, time
        );
        return goalie.control();
    }
    return Control();
}


Control AI::update_shooter(
    double time, GameState::Robot & robot, GameState::Ball & ball
){
    Eigen::Vector2d robot_position( 
        robot.position.getX(), robot.position.getY()
    );
    double robot_orientation = robot.orientation.getSignedValue();
    Eigen::Vector2d ball_position = Eigen::Vector2d(
        ball.position.getX(), ball.position.getY()
    );

    if (robot.isOk()) {
        if( shooter.is_static() ){
            shooter.go_to_shoot( 
                robot_position, ball_position, robot_orientation, time
            );
            return Control();
        }
        shooter.update(
            ball_position, robot_position, robot_orientation, time
        );
        return shooter.control();
    }
}


AI::AI(AIVisionClient *vision, AICommander *commander)
: vision(vision), commander(commander)
{
    running = true;
}


void AI::tick()
{
    double time = TimeStamp::now().getTimeMS()/1000.0;

    auto gameState = vision->getGameState();
   
    #ifdef SSL_SIMU
        int goalie_id = 0; 
        int shooter_id = 1;
    #else
        int goalie_id = 5; 
        int shooter_id = 8;
    #endif

    auto goalie_robot = gameState.robots[GameState::Ally][goalie_id];
    auto shooter_robot = gameState.robots[GameState::Ally][shooter_id];
    
    auto ball = gameState.ball;

    Control ctrl_goalie = update_goalie( time, goalie_robot, ball );
    prepare_to_send_control( goalie_id, ctrl_goalie );

    Control ctrl_shooter = update_shooter( time, shooter_robot, ball );
    prepare_to_send_control( shooter_id, ctrl_shooter );

    commander->flush();
}

void AI::run()
{
    double period = 1/100.0;    // 100 hz
    auto lastTick = TimeStamp::now();


    double robot_radius = 0.1;
    #ifdef SSL_SIMU
        // SSL SIMUL
        Eigen::Vector2d left_post_position( -4.5, -0.5 );
        Eigen::Vector2d right_post_position( -4.50, 0.5 );
        Eigen::Vector2d goal_center = (
            left_post_position + right_post_position
        )/2;
        Eigen::Vector2d waiting_goal_position(
            goal_center + Eigen::Vector2d(0.3, 0.0)
        );
        // PID for translation
        double p_translation = 0.2; 
        double i_translation = .00;
        double d_translation = .0;
        // PID for orientation
        double p_orientation = 0.4;
        double i_orientation = 0.0;
        double d_orientation = 0.0;
    #else
        // SSL QUALIF
        Eigen::Vector2d left_post_position( -2.8, -0.31 );
        Eigen::Vector2d right_post_position( -2.80, 0.29 );
        Eigen::Vector2d goal_center = (
            left_post_position + right_post_position
        )/2;
        Eigen::Vector2d waiting_goal_position(
            goal_center + Eigen::Vector2d(0.3, 0.0)
        );
        // PID for translation
        double p_translation = 0.01; 
        double i_translation = .00;
        double d_translation = .0;
        // PID for orientation
        double p_orientation = 0.02;
        double i_orientation = 0.0;
        double d_orientation = 0.0;
    #endif
    goalie.init(   
        left_post_position, right_post_position, 
        waiting_goal_position, robot_radius
    );
    goalie.set_translation_pid( 
        p_translation, i_translation, d_translation
    );
    goalie.set_orientation_pid(
        p_orientation, i_orientation, d_orientation
    );
    
    shooter.init(goal_center, robot_radius);
    goalie.set_translation_pid(
        p_translation, i_translation, d_translation
    );
    goalie.set_orientation_pid(
        p_orientation, i_orientation, d_orientation
    );

    while (running) {
        auto now = TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = period - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }
        lastTick = TimeStamp::now();
        tick();
    }
}

void AI::stop()
{
    running = false;
}
}
