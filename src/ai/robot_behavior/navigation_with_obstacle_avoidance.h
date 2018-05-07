#ifndef __ROBOT_BEHAVIOR__NAVIGATION_WITH_OBSTACLE_AVOIDANCE__H__
#define __ROBOT_BEHAVIOR__NAVIGATION_WITH_OBSTACLE_AVOIDANCE__H__

#include "robot_behavior.h"
#include <AiData.h>

namespace RhobanSSL
{
namespace Robot_behavior {

/*
 * This is an implementation of the article : 
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class Navigation_with_obstacle_avoidance : public RobotBehavior {
    private:

        Vector2d position;
        ContinuousAngle angle;

        double radius_of_limit_cycle;
        Vector2d limit_cycle_direction;
        ContinuousAngle angular_error;
        ContinuousAngle desired_angular_velocity;
        bool obstacle_avoidance_is_activated;

        void determine_the_closest_obstacle();
        bool do_we_activate_obstacle_avoidance();
        void compute_the_radius_of_limit_cycle();
        void compute_the_limit_cycle_direction();
        void convert_cycle_direction_to_linear_and_angular_velocity();
        void convert_position_error_to_linear_and_angular_velocity();

    public:
        Navigation_with_obstacle_avoidance( Ai::AiData & ai_data ); 

    public:
        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;

        void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

};

};
}; //Namespace Rhoban

#endif
