#ifndef __ROBOT_BEHAVIOR__NAVIGATION_WITH_OBSTACLE_AVOIDANCE__H__
#define __ROBOT_BEHAVIOR__NAVIGATION_WITH_OBSTACLE_AVOIDANCE__H__

#include "robot_behavior.h"
#include "position_follower.h"
#include <AiData.h>

namespace RhobanSSL {
namespace Robot_behavior {

/*
 * This is an implementation of the article : 
 * "Orbital Obstavle Avoidance Algorithm for reliable and on-line mobile robot navigation", Lounis Adouane, LASMEA.
 */
class Navigation_with_obstacle_avoidance :
    public ConsignFollower 
{
    private:
        PositionFollower position_follower;

        Vector2d target_position;
        ContinuousAngle target_angle;

        double min_time_collision;
        int closest_robot;

        double radius_of_limit_cycle;
        Vector2d limit_cycle_direction;

        struct Obstacle_point_of_view {
            rhoban_geometry::Point robot_linear_position;
            Vector2d robot_linear_velocity;
            rhoban_geometry::Point target_linear_position;
            
            Vector2d limit_cycle_direction;
        };
        Obstacle_point_of_view obstacle_point_of_view;
        double sign_of_avoidance_rotation;
        Control avoidance_control;

        void determine_the_closest_obstacle();
        void compute_the_radius_of_limit_cycle();
        void compute_the_limit_cycle_direction();
        void convert_cycle_direction_to_linear_and_angular_velocity();

    public:
        Navigation_with_obstacle_avoidance(
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

        virtual void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

};

};
}; //Namespace Rhoban

#endif
