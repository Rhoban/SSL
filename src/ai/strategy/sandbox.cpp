#include "sandbox.h"

#include <robot_behavior/position_follower.h>
#include <robot_behavior/goalie.h>
#include <robot_behavior/shooter.h>
#include <robot_behavior/do_nothing.h>

namespace RhobanSSL {
namespace Strategy {

const std::string Sandbox::name="sandbox";

int Sandbox::min_robots() const {
    return 2;        
}

int Sandbox::max_robots() const {
    return -1;
}


Sandbox::Sandbox(Ai::AiData & ai_data):
    Strategy(ai_data)
{
}

void Sandbox::start(double time){
    DEBUG("START SANDBOX");
    behavior_has_been_assigned = false;
}

void Sandbox::stop(double time){
    DEBUG("STOP SANDBOX");
}

void Sandbox::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( ! behavior_has_been_assigned ){
        int shooter_id = robot_id(0);
        //int shooter_id = TeamId::shooter_id;      
        int follower_id = robot_id(1);
        //int follower_id = TeamId::follower_id;
        for(
            int id : get_robot_ids()
        ){
            assign_behavior(
                id, std::shared_ptr<Robot_behavior::RobotBehavior>(
                    new Robot_behavior::DoNothing(ai_data)
                )
            );
        }

        DEBUG("SANDBOX ASSIGN BEHAVIOR");
        Robot_behavior::PositionFollower* follower = new Robot_behavior::PositionFollower(
            ai_data, time, dt
        );
        const Ai::Robot & robot_follower = ai_data.robots[
            Vision::Ally
        ][follower_id];
        Vector2d follower_position(
            robot_follower.get_movement().linear_position(time).getX(),
            robot_follower.get_movement().linear_position(time).getY()
        );
        follower->set_following_position(
            follower_position, ContinuousAngle(M_PI/2.0)
        );
        follower->set_translation_pid(
            ai_data.constants.p_translation,
            ai_data.constants.i_translation, 
            ai_data.constants.d_translation
        );
        follower->set_orientation_pid(
            ai_data.constants.p_orientation, ai_data.constants.i_orientation, 
            ai_data.constants.d_orientation
        );
        follower->set_limits(
            ai_data.constants.translation_velocity_limit,
            ai_data.constants.rotation_velocity_limit
        );
        assign_behavior( 
            follower_id, std::shared_ptr<
                Robot_behavior::RobotBehavior
            >( follower )
        ); 

        // We create a goalie :    
        Robot_behavior::Goalie* goalie = new Robot_behavior::Goalie(
            ai_data,
            ai_data.constants.left_post_position, 
            ai_data.constants.right_post_position, 
            ai_data.constants.waiting_goal_position, 
            ai_data.constants.penalty_rayon, 
            ai_data.constants.robot_radius,
            time, dt
        );
        goalie->set_translation_pid( 
            ai_data.constants.p_translation,
            ai_data.constants.i_translation, 
            ai_data.constants.d_translation
        );
        goalie->set_orientation_pid(
            ai_data.constants.p_orientation,
            ai_data.constants.i_orientation, 
            ai_data.constants.d_orientation
        );
        goalie->set_limits(
            ai_data.constants.translation_velocity_limit,
            ai_data.constants.rotation_velocity_limit
        );
        assign_behavior(
            get_goalie(), std::shared_ptr<
                Robot_behavior::RobotBehavior
            >( goalie )
        ); 

        #if 1
        // We create a shooter :
        Robot_behavior::Shooter* shooter = new Robot_behavior::Shooter(
            ai_data,
            ai_data.constants.goal_center,
            ai_data.constants.robot_radius,
            ai_data.constants.front_size,
            ai_data.constants.radius_ball,
            ai_data.constants.translation_velocity,
            ai_data.constants.translation_acceleration,
            ai_data.constants.angular_velocity,
            ai_data.constants.angular_acceleration,
            ai_data.constants.calculus_step,
            time, dt
        );
        shooter->set_translation_pid(
            ai_data.constants.p_translation,
            ai_data.constants.i_translation, 
            ai_data.constants.d_translation
        );
        shooter->set_orientation_pid(
            ai_data.constants.p_orientation,
            ai_data.constants.i_orientation, 
            ai_data.constants.d_orientation
        );
        shooter->set_limits(
            ai_data.constants.translation_velocity_limit,
            ai_data.constants.rotation_velocity_limit
        );
        const Ai::Ball & ball = ai_data.ball;
        Vector2d ball_position(
            ball.get_movement().linear_position(time).getX(),
            ball.get_movement().linear_position(time).getY()
        );
        const Ai::Robot & robot = ai_data.robots[Vision::Ally][shooter_id];
        Vector2d robot_position(
            robot.get_movement().linear_position(time).getX(),
            robot.get_movement().linear_position(time).getY()
        );
        double robot_orientation(
            robot.get_movement().angular_position(time).value()
        );
        shooter->go_to_shoot( 
            ball_position, robot_position, robot_orientation, 
            time, dt 
        );
        assign_behavior( 
            shooter_id, std::shared_ptr<
                Robot_behavior::RobotBehavior
            >( shooter )
        ); 
        #endif

        behavior_has_been_assigned = true;
    }
}

Sandbox::~Sandbox(){
}

}
}
