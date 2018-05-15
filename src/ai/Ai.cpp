#include "Ai.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <cmath>
#include <unistd.h>
#include <robot_behavior/do_nothing.h>
#include <manager/Manual.h>
#include <manager/Match.h>
#include <physic/MovementSample.h>
#include <math/vector2d.h>
#include <physic/constants.h>
#include <core/print_collection.h>

namespace RhobanSSL
{

        
void AI::check_time_is_coherent() const {
    #ifndef NDEBUG
    for( unsigned int i=0; i<ai_data.all_robots.size(); i++ ){
        assert(
            ai_data.all_robots.at(i).second->get_movement().last_time() - 0.000001 <= ai_data.time
       );
    }
    #endif
}

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

void AI::prevent_collision( int robot_id, Control & ctrl ){
    const Ai::Robot & robot = ai_data.robots.at(Vision::Team::Ally).at(robot_id);
    
    const Vector2d & ctrl_velocity = ctrl.velocity_translation;
    Vector2d robot_velocity = robot.get_movement().linear_velocity( ai_data.time );

    bool collision_is_detected = false;
    
    std::list< std::pair<int, double> > collisions_with_ctrl = ai_data.get_collisions(
        robot_id, ctrl_velocity
    );
    for( const std::pair<int, double> & collision : collisions_with_ctrl ){
        double time_before_collision = collision.second;
        double ctrl_velocity_norm = ctrl_velocity.norm();
        double time_to_stop = ctrl_velocity_norm/(
            ai_data.constants.security_acceleration_ratio
            *
            ai_data.constants.translation_acceleration_limit
        );
        if( time_before_collision <= time_to_stop and ctrl_velocity_norm > EPSILON_VELOCITY ){
            collision_is_detected = true;
        }
    }

    std::list< std::pair<int, double> > collisions_with_movement = ai_data.get_collisions(
        robot_id, robot_velocity
    );
    for( const std::pair<int, double> & collision : collisions_with_movement ){
        double time_before_collision = collision.second;
        double robot_velocity_norm = ctrl_velocity.norm();
        double time_to_stop = robot_velocity_norm/(
            ai_data.constants.security_acceleration_ratio
            *
            ai_data.constants.translation_acceleration_limit
        );
        if( time_before_collision <= time_to_stop and robot_velocity_norm > EPSILON_VELOCITY ){
            collision_is_detected = true;
        }
    }

    if( collision_is_detected ){
        double robot_velocity_norm = robot_velocity.norm();
        double velocity_increase = 0.0;
        if( robot_velocity_norm > 0 ){
            double velocity_increase = ( 1 - ai_data.dt * ai_data.constants.translation_acceleration_limit/robot_velocity_norm );
            if( velocity_increase < 0.0 ){
                velocity_increase = 0.0;
            }else{
            }
        }
        ctrl.velocity_translation = robot_velocity * velocity_increase;
    } 

#if 0
    //TODO improve the loop to work fast
    for( const std::pair< std::pair<int,int>, double > & elem : ai_data.table_of_collision_times ){
        const std::pair<int ,int> & collision = elem.first;
        Ai::Robot* robot = 0;
        if( 
            ( ai_data.all_robots[ collision.first ].second->id() == robot_id )
            and
            ( ai_data.all_robots[ collision.first ].first == Vision::Team::Ally )
        ){
            robot = ai_data.all_robots[ collision.first ].second;
        }
        if( 
            ( ai_data.all_robots[ collision.second ].second->id() == robot_id )
            and
            ( ai_data.all_robots[ collision.first ].first == Vision::Team::Ally )
        ){
            robot = ai_data.all_robots[ collision.second ].second;
        }
        if( robot ){ 
            //DEBUG("We stop Robot " << robot_id << " to prevent collision.");
            double time_before_collision = elem.second;
            //DEBUG( "time before collision : " << time );
            Vector2d velocity = robot->get_movement().linear_velocity( ai_data.time );
            double velocity_norm = velocity.norm();
            double time_to_stop = velocity.norm()/(
                ai_data.constants.security_acceleration_ratio
                *
                ai_data.constants.translation_acceleration_limit
            );
            // DEBUG("velo : " << velocity);
            // DEBUG("time to stop : " << time_to_stop << "accele : " << ai_data.constants.translation_acceleration_limit );
            if( time_before_collision <= time_to_stop and velocity_norm > EPSILON_VELOCITY ){
                double velocity_increase = ( 1 - ai_data.dt * ai_data.constants.translation_acceleration_limit/velocity_norm );
                if( velocity_increase < 0.0 ){
                    velocity_increase = 0.0;
                    DEBUG("SOP" );
                }else{
                    DEBUG("Emergency stop -- time_befor_coll " << time_before_collision << ", time_to_top " << time_to_stop );
                }
                velocity_increase = 0.0;
                ctrl.velocity_translation = velocity * velocity_increase;
            } 
        }
    }
#endif
}


static MovementSample debug_mov(4);

void AI::send_control( int robot_id, const Control & ctrl ){
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

void AI::prepare_to_send_control( int robot_id, Control & ctrl ){
#if 0    
    if( robot_id == 5 ){
        debug_mov.insert(
            PositionSample(
                ai_data.time,
                vector2point(ctrl.velocity_translation),
                ctrl.velocity_rotation       
            )
        );
        DEBUG(
            "ROBOT : " 
                << ai_data.time << " " 
                << debug_mov.dt(0) << " "
                << Vector2d(debug_mov.linear_position(0)).norm() << " "
                << debug_mov.linear_velocity(0).norm() << " "
                << ai_data.dt << " " 
                << ai_data.robots[Vision::Ally][robot_id].get_movement().linear_velocity(ai_data.time).norm() << " "
                << ai_data.robots[Vision::Ally][robot_id].get_movement().linear_acceleration(ai_data.time).norm() << " "
         );
    }
#endif

    prevent_collision( robot_id, ctrl );
    ctrl = ctrl.relative_control(
        ai_data.robots[Vision::Ally][robot_id].get_movement().angular_position( ai_data.time ), ai_data.dt
    );
    limits_velocity(ctrl);
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
        Shared_data::Final_control & final_control = shared_data.final_control_for_robots[robot_id]; 
        if( final_control.is_disabled_by_viewer  ){
            final_control.control = Control::make_desactivated();
        }else if( ! final_control.is_manually_controled_by_viewer ){
            Ai::Robot & robot = ai_data.robots[team][robot_id];

            Robot_behavior::RobotBehavior & robot_behavior = *( 
                robot_behaviors[robot_id] 
            );
            final_control.control = update_robot( 
                robot_behavior, time, robot, ball
            );
            prepare_to_send_control( robot_id, final_control.control );
        } 
        send_control( robot_id, final_control.control );
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
       
        #ifndef NDEBUG
        check_time_is_coherent();
        #endif

        referee.update(current_time);
        strategy_manager->remove_invalid_robots();
        strategy_manager->update(current_time);
        strategy_manager->assign_behavior_to_robots(robot_behaviors, current_time, current_dt);
        share_data();
        //ai_data.compute_table_of_collision_times();
        //if( ai_data.table_of_collision_times.size() != 0 ){
        //   DEBUG( ai_data.table_of_collision_times );
        //}

        data >> shared_data;    
    
        update_robots( );
        
        data << shared_data;    
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
