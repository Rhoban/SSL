#ifndef __ROBOT_BEHAVIOR__A_STAR_PATH__H__
#define __ROBOT_BEHAVIOR__A_STAR_PATH__H__

#include "robot_behavior.h"
#include "navigation_with_obstacle_avoidance.h"
#include "position_follower.h"
#include <AiData.h>

namespace RhobanSSL {
namespace Robot_behavior {

/*
 * This is an implementation of the article : 
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class A_star_path : public RobotBehavior {
    private:
        Navigation_with_obstacle_avoidance navigation;

        Vector2d target_position;
        ContinuousAngle target_angle;

    public:
        A_star_path(
            Ai::AiData & ai_data, double time, double dt
        ); 

    protected:
        void update_control(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

    public:
        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );


        virtual Control control() const;

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit,
            double translation_acceleration_limit,
            double rotation_acceleration_limit
        );

        void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

};

};
}; //Namespace Rhoban

#endif
