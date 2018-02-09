#include "AI.h"
#include <timing/TimeStamp.hpp>
#include <cmath>
#include <unistd.h>

#define TRACKING_ROBOT_SECURITY_TIME .2
// Comment the following line if you are working with the real robot.
// If you are working with the grSim simulator, don't comment.
#define SSL_SIMU


#ifdef SSL_SIMU
    #define ROTATION_VELOCITY_LIMIT -1.0
    #define TRANSLATION_VELOCITY_LIMIT -1.0
#else
    //#define ROTATION_VELOCITY_LIMIT 3.0
    //#define TRANSLATION_VELOCITY_LIMIT 2.0
    #define ROTATION_VELOCITY_LIMIT 20.0
    #define TRANSLATION_VELOCITY_LIMIT 8.0
#endif



using namespace Utils::Timing;

namespace RhobanSSL
{

Eigen::Vector2d point_to_eigen( const Point & point ){
    return Eigen::Vector2d( point.getX(), point.getY() );
}


void limits_velocity( Control & ctrl ){
    if( TRANSLATION_VELOCITY_LIMIT > 0.0 ){
        if( ctrl.velocity_translation.norm() > TRANSLATION_VELOCITY_LIMIT ){
            ctrl.velocity_translation = Eigen::Vector2d(0.0, 0.0);
            std::cerr << "WARNING : we reached the "
                "limit translation velocity !" << std::endl;
        }
    }
    if( ROTATION_VELOCITY_LIMIT > 0.0 ){
        if( std::abs( ctrl.velocity_rotation ) > ROTATION_VELOCITY_LIMIT ){
            ctrl.velocity_rotation = 0.0;
            std::cerr << "WARNING : we reached the "
                "limit rotation velocity !" << std::endl;
        }
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
    if( ctrl.kick and enable_kicking ){
    
        commander->kick();
    }
    #endif

    #ifdef SSL_SIMU
        int map_id;
        map_id = robot_id;
    #else
        int map_id;
        if( robot_id == 8 ){
            map_id = 1;
        }else{
            map_id = 0;
        }
    #endif

    
    if ( !ctrl.ignore) {
        if( ! ctrl.active ){
            commander->set(
                map_id, true, 0.0, 0.0, 0.0 
            );
        }else{
            commander->set(
                map_id, true, 
                ctrl.velocity_translation[0], sign_y*ctrl.velocity_translation[1], 
                ctrl.velocity_rotation
            );
        }
    }
}

Control AI::update_robot( 
    RobotBehavior & robot_behavior,
    double time, Ai::Robot & robot, Ai::Ball & ball
){
    if( robot.isOk() ){
        robot_behavior.update(time, robot, ball);
        return robot_behavior.control();
    }else{
        return Control::make_desactived();
    }
    return Control::make_ignored();
}

AI::AI(Data& data, AICommander *commander)
: time_sync(0.0), data(data), commander(commander) 
{
    running = true;
}


void AI::tick()
{
    double time = TimeStamp::now().getTimeMS()/1000.0 + time_sync;

    referee.update(time);
    
    Vision::VisionData visionData;
    data >> visionData;

/*
    DEBUG("");
    visionData.checkAssert(time);
    DEBUG("");

    if( time_sync == 0 ){
        if( visionData.older_time() == 0 ){
            DEBUG("");
            return;
        }
        time_sync = visionData.older_time() - time;
        DEBUG("time_sync :  " << time_sync);
        assert( time_sync < 0 ); 
        return;
    } 
*/

    game_state.update( visionData );
   
    #ifdef SSL_SIMU
        int goalie_id = 5; 
        int shooter_id = 0;
    #else
        int goalie_id = 8; 
        int shooter_id = 5;
    #endif

    auto ball = game_state.ball;

    auto goalie_robot = game_state.robots[Vision::Ally][goalie_id];
    //DEBUG( "goalie position : " << goalie_robot.position );
    //DEBUG( "goalie orientation : " << goalie_robot.orientation );

    Control ctrl_goalie = update_robot( goalie, time, goalie_robot, ball );
    prepare_to_send_control( goalie_id, ctrl_goalie );

    auto shooter_robot = game_state.robots[Vision::Ally][shooter_id];
    //DEBUG( "position : " << shooter_robot.position );
    //DEBUG( "orientation : " << shooter_robot.orientation );
    
    Control ctrl_shooter = update_robot( shooter, time, shooter_robot, ball );
    prepare_to_send_control( shooter_id, ctrl_shooter );

    commander->flush();
}

void AI::run()
{
    double period = 1/100.0;    // 100 hz
    auto lastTick = TimeStamp::now();


    double robot_radius = 0.09;
    double radius_ball = 0.04275/2.0;
    double translation_velocity_limit = TRANSLATION_VELOCITY_LIMIT;
    double rotation_velocity_limit = ROTATION_VELOCITY_LIMIT;

    #ifdef SSL_SIMU
        //DEBUG("SIMULATION MODE ACTIVATED");
        // SSL SIMUL
        
        double front_size = .06;
        
        Eigen::Vector2d left_post_position( -4.5, -0.5 );
        Eigen::Vector2d right_post_position( -4.50, 0.5 );
        Eigen::Vector2d goal_center = (
            left_post_position + right_post_position
        )/2;
        Eigen::Vector2d waiting_goal_position(
            goal_center + Eigen::Vector2d(0.0, 0.0)
        );
        // PID for translation
        double p_translation = 0.01; 
        double i_translation = .0;
        double d_translation = .0;
        // PID for orientation
        double p_orientation = 0.01;
        double i_orientation = 0.0;
        double d_orientation = 0.0;

        double translation_velocity = 3;
        double translation_acceleration = 12.0;
        double angular_velocity = 2.0*M_PI;  
        double angular_acceleration = 8*M_PI;

        double calculus_step = 0.0001;
        enable_kicking = false;

        double penalty_rayon = 1.0; // penalty rayon for the goalie
    #else
        DEBUG("REAL MODE ACTIVATED");
        // SSL QUALIF
        Eigen::Vector2d left_post_position( 0., -0.29 );
        Eigen::Vector2d right_post_position( 0., 0.29 );
        Eigen::Vector2d goal_center = (
            left_post_position + right_post_position
        )/2;
        Eigen::Vector2d waiting_goal_position(
            goal_center + Eigen::Vector2d(0.3, 0.0)
        );
        // PID for translation
        double p_translation = 0.02; 
        double i_translation = .01;
        double d_translation = .0;
        // PID for orientation
        double p_orientation = 0.02;
        double i_orientation = 0.001;
        double d_orientation = 0.0;

        double translation_velocity = 0.5;
        double translation_acceleration = 1.;
        double angular_velocity = 1.0;  
        double angular_acceleration = 5.;
        double calculus_step = 0.0001;
        enable_kicking = true;

        double penalty_rayon = 10.0; // For the goalie
    #endif
    goalie.init(   
        left_post_position, right_post_position, 
        waiting_goal_position, 
        penalty_rayon, robot_radius
    );
    goalie.set_translation_pid( 
        p_translation*2, 2*i_translation, d_translation
    );
    goalie.set_orientation_pid(
        p_orientation, i_orientation, d_orientation
    );
    goalie.set_limits(
        TRANSLATION_VELOCITY_LIMIT, ROTATION_VELOCITY_LIMIT 
    );
    
    shooter.init(
        goal_center, robot_radius,
        front_size, radius_ball,
        translation_velocity,
        translation_acceleration,
        angular_velocity,
        angular_acceleration,
        calculus_step
    );
    shooter.set_translation_pid(
        p_translation, i_translation, d_translation
    );
    shooter.set_orientation_pid(
        p_orientation, i_orientation, d_orientation
    );
    shooter.set_limits(
        TRANSLATION_VELOCITY_LIMIT, ROTATION_VELOCITY_LIMIT 
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
