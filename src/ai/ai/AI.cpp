#include "AI.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <cmath>
#include <unistd.h>
#include "robot_behavior/position_follower.h"
#include "robot_behavior/goalie.h"
#include "robot_behavior/shooter.h"
#include "robot_behavior/do_nothing.h"


using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace RhobanSSL
{


#ifdef SSL_SIMU
const int TeamId::goalie_id = 5; 
const int TeamId::shooter_id = 0;
const int TeamId::follower_id = 3;
#else
const int TeamId::goalie_id = 8; 
const int TeamId::shooter_id = 5;
const int TeamId::follower_id = 3;
#endif


Eigen::Vector2d point_to_eigen( const Point & point ){
    return Eigen::Vector2d( point.getX(), point.getY() );
}


void AI::limits_velocity( Control & ctrl ) const {
    if( constants.translation_velocity_limit > 0.0 ){
        if( 
            ctrl.velocity_translation.norm() > 
            constants.translation_velocity_limit 
        ){
            ctrl.velocity_translation = Eigen::Vector2d(0.0, 0.0);
            std::cerr << "WARNING : we reached the "
                "limit translation velocity !" << std::endl;
        }
    }
    if( constants.rotation_velocity_limit > 0.0 ){
        if( 
            std::abs( ctrl.velocity_rotation.value() ) > 
            constants.rotation_velocity_limit 
        ){
            ctrl.velocity_rotation = 0.0;
            std::cerr << "WARNING : we reached the "
                "limit rotation velocity !" << std::endl;
        }
    }
}

void AI::prepare_to_send_control(
    Vision::Team team, int robot_id, Control ctrl 
){
    limits_velocity(ctrl);

    #ifdef SSL_SIMU
        double sign_y = 1.0;
    #else
        double sign_y = -1.0;
    #endif

    AICommander * commander;
    if( team == Vision::Ally ){
        commander = commander_yellow;
    }else{
        commander = commander_blue;
    }
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
                ctrl.velocity_rotation.value()
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
        Control ctrl = robot_behavior.control();
        return ctrl; 
    }else{
        return Control::make_desactivated();
    }
    return Control::make_ignored();
}

void AI::stop_all_robots(){
    for( auto team : {Vision::Ally, Vision::Opponent} ){
        for( int k=0; k<Vision::Robots; k++ ){
            robot_behaviors[team][k] = std::shared_ptr<
                RobotBehavior
            >(
                new DoNothing()
            );
        }
    }
}


void AI::try_to_synchronize_time(){
    if( start_waiting_time_for_synchro < 0 ){
        start_waiting_time_for_synchro = current_time;
    }
    if(
        current_time - start_waiting_time_for_synchro > 
        waiting_time_for_synchro
    ){
        DEBUG("TIME SYNCHRONIZATION");
        time_synchro = true;
    }
}

bool AI::time_is_synchronized() const {
    return time_synchro;
}
                
void AI::assign_behavior_to_robots(){
    DEBUG("ASSIGN BEHAVIOR");


    PositionFollower* follower = new PositionFollower(
        current_time, current_dt
    );
    Vision::Team follower_team = Vision::Opponent;
    const Ai::Robot & robot_follower = game_state.robots[
        follower_team
    ][TeamId::follower_id];
    Eigen::Vector2d follower_position(
        robot_follower.get_movement().linear_position(current_time).getX(),
        robot_follower.get_movement().linear_position(current_time).getY()
    );
    follower->set_following_position(
        follower_position, ContinuousAngle(M_PI/2.0)
    );
    follower->set_translation_pid(
        constants.p_translation, constants.i_translation, 
        constants.d_translation
    );
    follower->set_orientation_pid(
        constants.p_orientation, constants.i_orientation, 
        constants.d_orientation
    );
    follower->set_limits(
        constants.translation_velocity_limit,
        constants.rotation_velocity_limit
    );
    robot_behaviors[follower_team][TeamId::follower_id] = std::shared_ptr<
        RobotBehavior
    >( follower ); 

    // We create a goalie :    
    Goalie* goalie = new Goalie(
        constants.left_post_position, constants.right_post_position, 
        constants.waiting_goal_position, 
        constants.penalty_rayon, constants.robot_radius,
        current_time, current_dt
    );
    goalie->set_translation_pid( 
        constants.p_translation, constants.i_translation, 
        constants.d_translation
    );
    goalie->set_orientation_pid(
        constants.p_orientation, constants.i_orientation, 
        constants.d_orientation
    );
    goalie->set_limits(
        constants.translation_velocity_limit,
        constants.rotation_velocity_limit
    );
    robot_behaviors[Vision::Ally][TeamId::goalie_id] = std::shared_ptr<
        RobotBehavior
    >( goalie ); 

#if 1
    // We create a shooter :
    Shooter* shooter = new Shooter(
        constants.goal_center, constants.robot_radius,
        constants.front_size, constants.radius_ball,
        constants.translation_velocity,
        constants.translation_acceleration,
        constants.angular_velocity,
        constants.angular_acceleration,
        constants.calculus_step,
        current_time, current_dt
    );
    shooter->set_translation_pid(
        constants.p_translation, constants.i_translation, 
        constants.d_translation
    );
    shooter->set_orientation_pid(
        constants.p_orientation, constants.i_orientation, 
        constants.d_orientation
    );
    shooter->set_limits(
        constants.translation_velocity_limit,
        constants.rotation_velocity_limit
    );
    const Ai::Ball & ball = game_state.ball;
    Eigen::Vector2d ball_position(
        ball.get_movement().linear_position(current_time).getX(),
        ball.get_movement().linear_position(current_time).getY()
    );
    const Ai::Robot & robot = game_state.robots[Vision::Ally][TeamId::shooter_id];
    Eigen::Vector2d robot_position(
        robot.get_movement().linear_position(current_time).getX(),
        robot.get_movement().linear_position(current_time).getY()
    );
    double robot_orientation(
        robot.get_movement().angular_position(current_time).value()
    );
    shooter->go_to_shoot( 
        ball_position, robot_position, robot_orientation, 
        current_time, current_dt 
    );
    robot_behaviors[Vision::Ally][TeamId::shooter_id] = std::shared_ptr<
        RobotBehavior
    >( shooter ); 
#endif

}


AI::AI(
    Data& data, 
    AICommander *commander_yellow,
    AICommander *commander_blue
): 
    data(data), 
    commander_yellow(commander_yellow),
    commander_blue(commander_blue),
    time_synchro(false),
    waiting_time_for_synchro(1.5),
    start_waiting_time_for_synchro(-1),
    current_dt(0.0),
    constants(game_state.constants), machine(game_state, game_state)
{
    running = true;
   
    stop_all_robots();
 
    machine
        .add_state( "init" )
        .add_state(
            "time_synchronisation",
            [&](
                Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                this->try_to_synchronize_time();
            }
        )
        .add_state( "time_is_synchronized" )
        .add_edge(
            "starting_synchronisation", 
            "init", "time_synchronisation" 
        )
        .add_edge(
            "synchronisation_finalisation", 
            "time_synchronisation", "time_is_synchronized",
            [&](
                const Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                return this->time_is_synchronized();
            }
        )
        .add_init_state( "init" )
    ;

    run_number_old = 0;

    machine
        .add_state( "wait_for_time_synchro" )
        .add_state(
            "wait_for_robot_update"
        )
        .add_state(
            "robot_is_updated"
        )
        .add_edge(
            "stop_all_robots",
            "init", "wait_for_time_synchro",
            [&](
                const Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                return true;
            },
            [&](
                Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                this->stop_all_robots();   
            }
        )
        .add_edge(
            "assign_role_to_robot",
            "wait_for_time_synchro", "wait_for_robot_update",
            [&](
                const Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                return machine.is_active( "time_is_synchronized" );
            },
            [&](
                Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                this->assign_behavior_to_robots();
            }
        )
        .add_edge(
            "update_robot",
            "wait_for_robot_update", "robot_is_updated",
            [&](
                const Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                return true;
            },
            [&](
                Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                this->update_robots( );
            }
        )
        .add_edge(
            "robot_waiting_new_run",
            "robot_is_updated", "wait_for_robot_update", 
            [&](
                const Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                if( run_number > run_number_old ){
                    run_number_old = run_number;
                    return true;
                }
                return false;
            }
        )
    ;
    machine.start();

    machine.export_to_file("machine.dot");
}


void AI::update_robots( ){
    double time =  this->current_time;
    Ai::Ball & ball = game_state.ball;

    for( auto team : {Vision::Ally, Vision::Opponent} ){
        for( int robot_id=0; robot_id<Vision::Robots; robot_id++ ){

            Ai::Robot & robot = game_state.robots[team][robot_id];

            RobotBehavior & robot_behavior = *( 
                robot_behaviors[team][robot_id] 
            );
            Control ctrl = update_robot( 
                robot_behavior, time, robot, ball
            ); 
#if 0 
            if(team == Vision::Ally && robot_id == TeamId::shooter_id){                
                DEBUG( "sample : " << robot.get_movement().get_sample() );
                DEBUG( "derivate : " << robot.get_movement().get_sample() );
                DEBUG( "linear position : " << robot.get_movement().linear_position(this->current_time) );
                DEBUG( "angular position : " << robot.get_movement().angular_position(this->current_time) );
                DEBUG( "linear velocity : " << robot.get_movement().linear_velocity(this->current_time) );
                DEBUG( "angular velocity : " << robot.get_movement().angular_velocity(this->current_time) );
                DEBUG( "linear acceleration : " << robot.get_movement().linear_acceleration(this->current_time) );
                DEBUG( "angular acceleration : " << robot.get_movement().angular_acceleration(this->current_time) );
                DEBUG( "ctrl : " << ctrl );
            }
#endif

            prepare_to_send_control( team, robot_id, ctrl );
        }
    }
    commander_yellow->flush();
    commander_blue->flush();
}

void AI::run(){
    double period = 1/60.0;    // 100 hz
    auto lastTick = TimeStamp::now();

    while (running) {
        auto now = TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = period - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }else{
            DEBUG("LAG");
        }
        lastTick = TimeStamp::now();
        current_dt = current_time;
        current_time = TimeStamp::now().getTimeMS()/1000.0;
        current_dt = current_time - current_dt;
        
        data >> visionData;

        //DEBUG( visionData );

        //DEBUG("");
        visionData.checkAssert(current_time);
        //DEBUG("");
        
        game_state.update( visionData );
        
        referee.update(current_time);

        machine.run( );
    }
}

void AI::stop()
{
    running = false;
}

    

}
