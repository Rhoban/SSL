#include "Ai.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <cmath>
#include <unistd.h>
#include <robot_behavior/do_nothing.h>
#include <manager/Manual.h>
#include <manager/Match.h>

namespace RhobanSSL
{


void AI::limits_velocity( Control & ctrl ) const {
    if( game_state.constants.translation_velocity_limit > 0.0 ){
        if( 
            ctrl.velocity_translation.norm() > 
            game_state.constants.translation_velocity_limit 
        ){
            ctrl.velocity_translation = Eigen::Vector2d(0.0, 0.0);
            std::cerr << "WARNING : we reached the "
                "limit translation velocity !" << std::endl;
        }
    }
    if( game_state.constants.rotation_velocity_limit > 0.0 ){
        if( 
            std::abs( ctrl.velocity_rotation.value() ) > 
            game_state.constants.rotation_velocity_limit 
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
        Control ctrl = robot_behavior.control();
        return ctrl; 
    }else{
        return Control::make_desactivated();
    }
    return Control::make_ignored();
}

void AI::init_robot_behaviors(){
    for( int k=0; k<Vision::Robots; k++ ){
        robot_behaviors[k] = std::shared_ptr<
            RobotBehavior
        >(
            new DoNothing()
        );
    }
}

AI::AI(
    Ai::Team team,
    Data& data, 
    AICommander *commander
):
    team(team), 
    data(data), 
    commander(commander),
    current_dt(0.0)
{
    running = true;
   
    init_robot_behaviors();
    std::vector<int> robot_ids( robot_behaviors.size() );
    int i = 0;
    for( auto elem : robot_behaviors ){
        robot_ids[i] = elem.first;
        i++;
    } 
    #ifdef SSL_SIMU
    int goalie_id = 5;
    #else
    int goalie_id = 8;
    #endif
    strategy_manager = std::shared_ptr<Manager::Manager>(
        //new Manager::Manual(game_state)
        new Manager::Match(game_state, referee)
    );
    strategy_manager->declare_goalie_id( goalie_id );
    strategy_manager->declare_team_ids( robot_ids );

}


void AI::update_robots( ){
    double time =  this->current_time;
    Ai::Ball & ball = game_state.ball;
    
    auto team = Vision::Ally;
    for( int robot_id=0; robot_id<Vision::Robots; robot_id++ ){

        Ai::Robot & robot = game_state.robots[team][robot_id];

        RobotBehavior & robot_behavior = *( 
            robot_behaviors[robot_id] 
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

        prepare_to_send_control( robot_id, ctrl );
    }
    commander->flush();
}

void AI::run(){
    double period = 1/60.0;    // 100 hz
    auto lastTick = rhoban_utils::TimeStamp::now();

    while (running) {
        auto now = rhoban_utils::TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = period - elapsed;
        if (toSleep > 0) {
            usleep(round(toSleep*1000000));
        }else{
            DEBUG("LAG");
        }
        lastTick = rhoban_utils::TimeStamp::now();
        current_dt = current_time;
        current_time = rhoban_utils::TimeStamp::now().getTimeMS()/1000.0;
        current_dt = current_time - current_dt;
        
        data >> visionData;

        //DEBUG( visionData );

        //DEBUG("");
        visionData.checkAssert(current_time);
        //DEBUG("");
        
        game_state.update( visionData );
        referee.update(current_time);
        strategy_manager->update(current_time);
        strategy_manager->assign_behavior_to_robots(robot_behaviors, current_time, current_dt);
        update_robots( );
    }
}

void AI::stop()
{
    running = false;
}

    
}
