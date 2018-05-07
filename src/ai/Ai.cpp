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
    if( ai_data.constants.translation_velocity_limit > 0.0 ){
        if( 
            ctrl.velocity_translation.norm() > 
            ai_data.constants.translation_velocity_limit 
        ){
            ctrl.velocity_translation = Vector2d(0.0, 0.0);
            std::cerr << "AI WARNING : we reached the "
                "limit translation velocity !" << std::endl;
        }
    }
    if( ai_data.constants.rotation_velocity_limit > 0.0 ){
        if( 
            std::fabs( ctrl.velocity_rotation.value() ) > 
            ai_data.constants.rotation_velocity_limit 
        ){
            ctrl.velocity_rotation = 0.0;
            std::cerr << "AI WARNING : we reached the "
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
            Vector2d velocity_translation = ai_data.team_point_of_view.from_basis(
                ctrl.velocity_translation
            ); 
            commander->set(
                map_id, true, 
                velocity_translation[0], sign_y*velocity_translation[1], 
                ctrl.velocity_rotation.value()
            );
        }
    }
}

Control AI::update_robot( 
    Robot_behavior::RobotBehavior & robot_behavior,
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
            Robot_behavior::RobotBehavior
        >(
            new Robot_behavior::DoNothing(ai_data)
        );
    }
}


AI::AI(
    std::string team_name,
    Ai::Team default_team,
    Data& data, 
    AICommander *commander
):
    team_name(team_name),
    default_team(default_team), 
    running(true),
    commander(commander),
    current_dt(0.0),
    data(data)
{
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

    ai_data.change_team_color(default_team);
    ai_data.team_name = team_name;
    strategy_manager = std::shared_ptr<Manager::Manager>(
        //new Manager::Manual(ai_data)
        new Manager::Match(ai_data, referee)
    );
    strategy_manager->declare_goalie_id( goalie_id );
    strategy_manager->declare_team_ids( robot_ids );

}


void AI::update_robots( ){

    commander->set_yellow( ai_data.team_color == Ai::Yellow  );

    double time =  this->current_time;
    Ai::Ball & ball = ai_data.ball;
    
    auto team = Vision::Ally;
    for( int robot_id=0; robot_id<Vision::Robots; robot_id++ ){

        Ai::Robot & robot = ai_data.robots[team][robot_id];

        Robot_behavior::RobotBehavior & robot_behavior = *( 
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
        
        ai_data.time = current_time,
        ai_data.dt = current_dt;

        data >> visionData;

        //DEBUG( visionData );

        //DEBUG("");
        visionData.checkAssert(current_time);
        //DEBUG("");
        
        ai_data.update( visionData );
        referee.update(current_time);
        strategy_manager->remove_invalid_robots();
        strategy_manager->update(current_time);
        strategy_manager->assign_behavior_to_robots(robot_behaviors, current_time, current_dt);
        share_data();
        update_robots( );
    }
}

void AI::stop()
{
    running = false;
}

void AI::share_data(){
    Data_from_ai data_from_ai;
    data_from_ai.team_color = ai_data.team_color;
    data << data_from_ai;
}
    
}
