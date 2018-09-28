/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Ai.h"
#include <rhoban_utils/timing/time_stamp.h>
#include <cmath>
#include <rhoban_utils/angle.h>
#include <unistd.h>
#include <robot_behavior/do_nothing.h>
#include <manager/Manual.h>
#include <manager/Match.h>
#include <physic/MovementSample.h>
#include <math/vector2d.h>
#include <physic/constants.h>
#include <core/print_collection.h>
#include <core/collection.h>
#include <manager/factory.h>
#include <debug.h>
#include <com/AICommanderReal.h>
#include <utility>

namespace RhobanSSL
{

float sign(float x) {
    if (x > 0) {
        return 1.0;
    }
    return -1.0;
}

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
#if 1
    if( ai_data.constants.translation_velocity_limit > 0.0 ){
        if(
            ctrl.velocity_translation.norm() >
            ai_data.constants.translation_velocity_limit
        ){
            ctrl.velocity_translation *= ai_data.constants.translation_velocity_limit/ctrl.velocity_translation.norm();
            std::cerr << "AI WARNING : we reached the "
                "limit translation velocity !" << std::endl;
        }
    }
    if( ai_data.constants.rotation_velocity_limit > 0.0 ){
        if(
            std::fabs( ctrl.velocity_rotation.value() ) >
            ai_data.constants.rotation_velocity_limit
        ){
            ctrl.velocity_rotation = ai_data.constants.rotation_velocity_limit*sign(ctrl.velocity_rotation.value());
            std::cerr << "AI WARNING : we reached the "
                "limit rotation velocity !" << std::endl;
        }
    }
#endif
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
        if( time_before_collision <= 0.1 ){
            collision_is_detected = false; //We try to desengage
        }
    }

    /* Prevent real collision */
    /* Uncomment for more safety */
    /*
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
    */
    if( collision_is_detected ){
        double robot_velocity_norm = robot_velocity.norm();
        double velocity_increase = 0.0;
        double err = 0.01;
        if( robot_velocity_norm > err ){
            velocity_increase = ( 1 - ai_data.dt * ai_data.constants.translation_acceleration_limit/robot_velocity_norm );
            if( velocity_increase < 0.0 ){
                velocity_increase = 0.0;
            }
            ctrl.velocity_translation = robot_velocity * velocity_increase;
        }else{
            // We want to go out the blockage situation
        }
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
    if( robot_id >= 8 ){ //HACK - becaus hardware doesn't support more than 8 robots
        return; //HACK
    } //HACK
    assert( robot_id < 8 ); //HACK !
    if ( !ctrl.ignore) {
        if( ! ctrl.active ){
            commander->set(
                robot_id, true, 0.0, 0.0, 0.0
            );
        }else{
            //if( robot_id == 1 ){
            //    DEBUG( "CTRL : " << ctrl );
            //}
            int kick = 0;
            if (ctrl.kick) kick = 1;
            else if (ctrl.chipKick) kick = 2;
            
            if(ctrl.tareOdom){
                commander->set(
                robot_id, true,
                ctrl.fix_translation[0], ctrl.fix_translation[1],
                ctrl.fix_rotation.value(),
                kick,
                ctrl.kickPower,
                ctrl.spin,
                ctrl.charge,
                ctrl.tareOdom

            );
                //DEBUG("TARE : " << ctrl.tareOdom<<" | "<<ctrl.fix_rotation);
            }else{
                commander->set(
                robot_id, true,
                ctrl.velocity_translation[0], ctrl.velocity_translation[1],
                ctrl.velocity_rotation.value(),
                kick,
                ctrl.kickPower,
                ctrl.spin,
                ctrl.charge,
                ctrl.tareOdom
            );
            }
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
    static_cast<PidControl &>(ctrl) = ctrl.relative_control(
        ai_data.robots[Vision::Ally][robot_id].get_movement().angular_position( ai_data.time ), ai_data.dt
    );
    limits_velocity(ctrl);
}

Control AI::update_robot(
    Robot_behavior::RobotBehavior & robot_behavior,
    double time, Ai::Robot & robot, Ai::Ball & ball
){
    if( robot.is_present_in_vision() ){
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
    std::string manager_name,
    std::string team_name,
    Ai::Team default_team,
    Data& data,
    AICommander *commander,
    const std::string & config_path,
    bool is_in_simulation
):
    team_name(team_name),
    default_team(default_team),
    is_in_simulation(is_in_simulation),
    running(true),
    ai_data( config_path, is_in_simulation, default_team ),
    commander(commander),
    current_dt(ai_data.constants.period),
    data(data)
{
    init_robot_behaviors();


    ai_data.change_team_color(default_team);
    ai_data.team_name = team_name;

    manual_manager = Manager::Factory::construct_manager(
        Manager::names::manual, ai_data, referee
    );

    setManager( manager_name );
}

std::vector<std::string> AI::getAvailableManagers()
{
    return list2vector( Manager::Factory::avalaible_managers() );
}

void AI::setManager(std::string managerName)
{
    std::vector<int> robot_ids( robot_behaviors.size() );
    int i = 0;
    for( auto elem : robot_behaviors ){
        robot_ids[i] = elem.first;
        i++;
    }

    int goalie_id = ai_data.constants.default_goalie_id;

    std::cout << "Setting the manager to: " << managerName << std::endl;
    if( managerName == Manager::names::manual ){
        strategy_manager = manual_manager;
    }else{
        strategy_manager = Manager::Factory::construct_manager(
            managerName, ai_data, referee
        );
    }
    manager_name = managerName;
    strategy_manager->declare_goalie_id( goalie_id );
    strategy_manager->declare_team_ids( robot_ids );
}

std::shared_ptr<Manager::Manager> AI::getManager() const
{
    return strategy_manager;
}

std::shared_ptr<Manager::Manager> AI::getManualManager()
{
    return manual_manager;
}

void AI::update_robots( ){

    commander->set_yellow( ai_data.team_color == Ai::Yellow  );

    double time =  this->current_time;
    Ai::Ball & ball = ai_data.ball;

    auto team = Vision::Ally;

    //auto timeindice1 = rhoban_utils::TimeStamp::now();
    for( int robot_id=0; robot_id<Vision::Robots; robot_id++ ){
        //auto timeindice0 = rhoban_utils::TimeStamp::now();
        Shared_data::Final_control & final_control = shared_data.final_control_for_robots[robot_id];
        Ai::Robot & robot = ai_data.robots[team][robot_id];
        Robot_behavior::RobotBehavior & robot_behavior = *(
            robot_behaviors[robot_id]
        );
        robot_behavior.update(time, robot, ball);

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

            robot.ordersSample.insert(SpeedTargetSample(ai_data.time, (int16_t)(commander->commands[robot_id].xSpeed*1000), (int16_t)(commander->commands[robot_id].ySpeed*1000), (int16_t)(commander->commands[robot_id].thetaSpeed*1000)));
            robot.movement->set_orders_sample(robot.ordersSample);
        }
        // if ((robot.getIncertitudeOdomTime(time) > 1.0) || (robot.getIncertitudeOdomTime(time) == 0.0)){
        //     final_control.control.fix_translation[0] = robot.movement->get_sample(0).linear_position().getX();
        //     final_control.control.fix_translation[1] = robot.movement->get_sample(0).linear_position().getY();
        //     final_control.control.fix_rotation       = rhoban_utils::normalizeRad(rhoban_utils::deg2rad(robot.movement->get_sample(0).angular_position().angle().getSignedValue()));
        //     final_control.control.tareOdom = true;
        //     robot.setOdomTime(time);
        // }
        if ((odomData.robots.at(Vision::Ally).at(robot_id).ageOdom(ai_data.time) > 1.0)&&(odomData.robots.at(Vision::Ally).at(robot_id).present == true)){
            DEBUG(robot_id);
            final_control.control.fix_translation[0] = robot.movement->get_sample(0).linear_position().getX();
            final_control.control.fix_translation[1] = robot.movement->get_sample(0).linear_position().getY();
            final_control.control.fix_rotation       = rhoban_utils::normalizeRad(rhoban_utils::deg2rad(robot.movement->get_sample(0).angular_position().angle().getSignedValue()));
            final_control.control.tareOdom = true;
            odomData.robots.at(Vision::Ally).at(robot_id).setLastUpdate(ai_data.time);    

        }
        else{
            final_control.control.tareOdom = false;
        }

        
        send_control( robot_id, final_control.control );
        //DEBUG("send_control time : " << robot_id << " : " << diffSec(timeindice0, rhoban_utils::TimeStamp::now())); //2.5 us
        //DEBUG(diffSec(ai_data.all_robots[team][robot_id]->lastUpdate, rhoban_utils::TimeStamp::now()));
    }
    //DEBUG("send_all_control time : " << diffSec(timeindice1, rhoban_utils::TimeStamp::now())); //15 us

}

void AI::run(){
    double period = ai_data.constants.period;
    auto lastTick = rhoban_utils::TimeStamp::now();

    while (running) {
        auto now = rhoban_utils::TimeStamp::now();
        double elapsed = diffSec(lastTick, now);
        double toSleep = period - elapsed;
        if (toSleep > 0)    {
            usleep(round(toSleep*1000000));
        }else{
            //DEBUG("LAG");
        }
        lastTick = rhoban_utils::TimeStamp::now();
        current_dt = current_time;
        current_time = rhoban_utils::TimeStamp::now().getTimeMS()/1000.0;
        current_dt = current_time - current_dt;

        //DEBUG("Current dt : " << current_dt); //16.7 ms, 60Hz


        ai_data.time = current_time,
        ai_data.dt = current_dt;

        #ifndef NDEBUG
        update_periodic_debug( current_time, 10.0 );
        #endif

        data >> visionData;

        //DEBUG( visionData );

        //DEBUG("");
        visionData.checkAssert(current_time);
        odomData.checkAssert(current_time);

        ai_data.update( visionData );
        ai_data.update( odomData );

        if( not(is_in_simulation) ){
            update_electronic_informations();
        }
        //print_electronic_info();

        #ifndef NDEBUG
        //check_time_is_coherent();
        #endif

        referee.update(current_time);

        if( manager_name != Manager::names::manual  ) { // HACK TOT REMOVEE !
            strategy_manager->change_team_and_point_of_view(
                referee.get_team_color( strategy_manager->get_team_name() ),
                referee.blue_have_it_s_goal_on_positive_x_axis()
            );
        }else{
            dynamic_cast<Manager::Manual*>(
                strategy_manager.get()
            )->define_goal_to_positive_axis(
                not( referee.blue_have_it_s_goal_on_positive_x_axis() )
            );
        }
        strategy_manager->change_ally_and_opponent_goalie_id(
            referee.blue_goalie_id(),
            referee.yellow_goalie_id()
        );

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

        data.edit_data_for_viewer(
            [this]( Data_for_viewer & data_for_viewer ){
                data_for_viewer.annotations.clear();
                this->get_annotations( data_for_viewer.annotations );
            }
        );

        // XXX: Flushing takes some time in real mode, and should be done in parallel
        // along with the computing of the AI
        //auto timeindice0 = rhoban_utils::TimeStamp::now();
        commander->flush();
        //DEBUG("Flush time : " << diffSec(timeindice0, rhoban_utils::TimeStamp::now())); //180 us 
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

Referee &AI::getReferee()
{
    return referee;
}

double AI::getCurrentTime()
{
    return ai_data.time;
}


RhobanSSLAnnotation::Annotations AI::get_robot_behavior_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;
    for( int robot_id=0; robot_id<Vision::Robots; robot_id++ ){
        const Robot_behavior::RobotBehavior & robot_behavior = *(
            robot_behaviors.at(robot_id)
        );
        annotations.addAnnotations( robot_behavior.get_annotations() );
    }
    return annotations;
}


void AI::get_annotations( RhobanSSLAnnotation::Annotations & annotations ) const {
    annotations.addAnnotations( getManager()->get_annotations() );
    annotations.addAnnotations(
        get_robot_behavior_annotations()
    );

    std::function<
        rhoban_geometry::Point (
            const rhoban_geometry::Point & p
        )
    > fct = [this]( const rhoban_geometry::Point & p ) {
        return this->ai_data.team_point_of_view.from_frame( p );
    };
    annotations.map_positions(fct);
}


void AI::update_electronic_informations(){
    RhobanSSL::Master *master = dynamic_cast<RhobanSSL::AICommanderReal*>(commander)->getMaster();
    auto team = Vision::Ally;
    for( unsigned int id=0; id<MAX_ROBOTS; id++ ){
        auto robot = master->robots[id];
        if( robot.isOk() ){
            Ai::Robot & robotai = ai_data.robots.at(team).at(id);
            robotai.infra_red = (robot.status.status & STATUS_IR) ? true : false;
            if(odomData.robots.at(team).at(id).present == false){
                odomData.robots.at(team).at(id).present = true;
            }
            odomData.robots.at(team).at(id).update(ai_data.time, rhoban_geometry::Point(((double)robot.status.xpos)/1000, ((double)robot.status.ypos)/1000), ContinuousAngle(((double)(robot.status.ang))/1000)); //ODOME
            //unsigned int odometry_index = 1; // TODO Faire une enum !
            //robotai.movement->set_sample(robotai.odom_data.movement, odometry_index);
        }
    }
}

void AI::print_electronic_info(){
    std::cout << "Electronic : " << std::endl;
    for( unsigned int id=0; id<Ai::Constants::NB_OF_ROBOTS_BY_TEAM; id++ ){
        std::cout << "robot id : " << id << " IR : " << ai_data.robots.at(Vision::Team::Ally).at(id).infra_red << std::endl;
    }
}

}
