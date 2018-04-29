#include "enseirb_project_wrapper.h"
#include <robot_behavior/do_nothing.h>
#include <robot_behavior/position_follower.h>
#include <robot_behavior/apply_enseirb_project_action.h>

namespace RhobanSSL {
namespace Strategy {

Enseirb_project_wrapper::Enseirb_project_wrapper(Ai::AiData & game_state):
    game_state(game_state),
    robots(0), nb_robots(0)
{
}

const std::string Enseirb_project_wrapper::name="enseirb_project_wrapper";

int Enseirb_project_wrapper::min_robots() const {
    return 0;
}
int Enseirb_project_wrapper::max_robots() const {
    return -1;
}

void Enseirb_project_wrapper::allocate_enseirb_data(){
    nb_robots = 0;
    for( auto team: {Vision::Ally, Vision::Opponent} ){
        nb_robots+=game_state.robots[team].size();
    }
    robot_actions = std::vector<Action>(get_robot_ids().size());
    robots = new Robot[nb_robots];
}

void Enseirb_project_wrapper::initialize_enseirb_data(){
    config.width = game_state.field.fieldWidth;
    config.height = game_state.field.fieldLength;
    config.goal_size = game_state.field.goalWidth;
    ball.radius = game_state.constants.radius_ball;
    DEBUG( 
        "ICI -- width : " << 
        config.width <<
        ", height : " << 
        config.height <<
        ", goal_size : " << 
        config.goal_size <<
        ", ball radius : " << 
        ball.radius
    );
    int i=0;
    for( auto team: {Vision::Ally, Vision::Opponent} ){
        for( const std::pair<int, Ai::Robot> & elem : game_state.robots[team] ){
            int id = elem.first;
            // index_2_robot[i] = std::pair<Vision::Team, int>( team, id );
            robot_2_index[std::pair<Vision::Team, int>( team, id )] = i;
            robots[i].id = id;
            robots[i].radius = game_state.constants.robot_radius;
            robots[i].team = (team == Vision::Ally)? TEAM_1:TEAM_2;
            //Robots[i].behaviour.id = // TODO 
            i+=1;
        }
    }
}

void Enseirb_project_wrapper::desallocate_enseirb_data(){
    delete robots;
    robots = 0;
}

void Enseirb_project_wrapper::start(double time){
    DEBUG("START ENSEIRB");
    allocate_enseirb_data();
    initialize_enseirb_data();
    // TODO
    setBehaviour(
        &config, robots, nb_robots, TEAM_1
    );

    behavior_has_been_assigned = false;
}

void Enseirb_project_wrapper::stop(double time){
    desallocate_enseirb_data();
    DEBUG("STOP ENSEIRB");
}

void Enseirb_project_wrapper::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        for( int i=0; i<get_robot_ids().size(); i++ ){
            RhobanSSL::Apply_enseirb_project_action *robot_behavior = new RhobanSSL::Apply_enseirb_project_action(
                robot_actions[i],
                time, dt
            ) ;

            robot_behavior->set_translation_pid(
                game_state.constants.p_translation,
                game_state.constants.i_translation, 
                game_state.constants.d_translation
            );
            robot_behavior->set_orientation_pid(
                game_state.constants.p_orientation, game_state.constants.i_orientation, 
                game_state.constants.d_orientation
            );
            robot_behavior->set_limits(
                game_state.constants.translation_velocity_limit,
                game_state.constants.rotation_velocity_limit
            );

            assign_behavior(
                robot_id(i), std::shared_ptr<RobotBehavior>( robot_behavior )
            );
        }
        behavior_has_been_assigned = true;
    }
}

void Enseirb_project_wrapper::update(double time){
    ContinuousAngle angular_position;
    ContinuousAngle angular_velocity;

    //update ball
    const RhobanSSL::Movement & mov = game_state.ball.get_movement();

    rhoban_geometry::Point linear_position = mov.linear_position(time );
    Vector2d linear_velocity = mov.linear_velocity(time);

    this->ball.x = linear_position.getX();
    this->ball.y = linear_position.getY();
    this->ball.vx = linear_velocity.getX();
    this->ball.vy = linear_velocity.getY();

    //update all robot positions
    for( auto team: {Vision::Ally, Vision::Opponent} ){
        for( const std::pair<int, Ai::Robot> & elem : game_state.robots[team] ){
            int id = elem.first;
            const Ai::Robot & robot = elem.second;
            const RhobanSSL::Movement & robot_mov = game_state.ball.get_movement();
            linear_position = robot_mov.linear_position(time );
            angular_position = robot_mov.angular_position(time);
            linear_velocity = robot_mov.linear_velocity(time);
            angular_velocity = robot_mov.angular_velocity(time);
            int i = robot_2_index.at( std::pair<Vision::Team, int>(team, id) );
            robots[i].x = linear_position.getX();
            robots[i].y = linear_position.getY(); 
            robots[i].t = angular_position.value();
            robots[i].vx = linear_velocity.getX();
            robots[i].vy = linear_velocity.getY(); 
            robots[i].vt = angular_velocity.value();
        }
    } 
        
    // get robot actions
    for( int i=0; i<get_robot_ids().size(); i++ ){
        Action action = getBehaviour(
            &config, robots, nb_robots, &ball, 
            robot_id(i)
        );
        robot_actions[i] = action;
    }
}

Enseirb_project_wrapper::~Enseirb_project_wrapper(){
    desallocate_enseirb_data();
}


}
}
