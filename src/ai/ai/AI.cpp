#include "AI.h"
#include <timing/TimeStamp.hpp>
#include <cmath>
#include <unistd.h>



using namespace Utils::Timing;

namespace RhobanSSL
{

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
        return robot_behavior.control();
    }else{
        return Control::make_desactived();
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
        DEBUG("TIME SYNCHRONIZation");
        time_synchro = true;
    }
}

bool AI::time_is_synchronized() const {
    return time_synchro;
}
                
void AI::assign_behavior_to_robots(){
    DEBUG("ASSIGN BEHAVIOR");
    
    goalie.init(
        constants.left_post_position, constants.right_post_position, 
        constants.waiting_goal_position, 
        constants.penalty_rayon, constants.robot_radius
    );
    goalie.set_translation_pid( 
        constants.p_translation*2, 2*constants.i_translation, 
        constants.d_translation
    );
    goalie.set_orientation_pid(
        constants.p_orientation, constants.i_orientation, 
        constants.d_orientation
    );
    goalie.set_limits(
        constants.translation_velocity_limit,
        constants.rotation_velocity_limit
    );
    
    shooter.init(
        constants.goal_center, constants.robot_radius,
        constants.front_size, constants.radius_ball,
        constants.translation_velocity,
        constants.translation_acceleration,
        constants.angular_velocity,
        constants.angular_acceleration,
        constants.calculus_step
    );
    shooter.set_translation_pid(
        constants.p_translation, constants.i_translation, 
        constants.d_translation
    );
    shooter.set_orientation_pid(
        constants.p_orientation, constants.i_orientation, 
        constants.d_orientation
    );
    shooter.set_limits(
        constants.translation_velocity_limit,
        constants.rotation_velocity_limit
    );

}


AI::AI(Data& data, AICommander *commander): 
    data(data), commander(commander),
    time_synchro(false),
    waiting_time_for_synchro(4),
    start_waiting_time_for_synchro(-1),
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
    machine
        .add_state( "wait_for_time_synchro" )
        .add_state(
            "update_robot",
            [&](
                Ai::AiData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                this->update_robots( );
            }
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
            "wait_for_time_synchro", "update_robot",
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
    ;
    machine.start();

    machine.export_to_file("machine.dot");
}


void AI::update_robots( ){
    double time =  this->current_time;
   
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

void AI::run(){
    double period = 1/100.0;    // 100 hz
    auto lastTick = TimeStamp::now();

    while (running) {
        auto now = TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = period - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }
        lastTick = TimeStamp::now();

        current_time = TimeStamp::now().getTimeMS()/1000.0;

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
