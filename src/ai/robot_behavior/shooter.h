#ifndef __SHOOTER__H__
#define __SHOOTER__H__

#include "robot_behavior.h"

namespace RhobanSSL {

struct Translation_for_shooting {
    Vector2d position_robot;
    Vector2d position_ball;
    Vector2d goal_center;
   
    double front_size;
    double radius_ball;
 
    Vector2d operator()(double u) const;
};

struct Rotation_for_shooting {
    double orientation;
    double end;

    double operator()(double u) const;
};

struct Translation_for_home {
    Vector2d position_robot;
    Vector2d position_home;
    
    Vector2d operator()(double u) const;
};

struct Rotation_for_home {
    double orientation;
    Vector2d position_ball;
    Vector2d position_robot;

    double operator()(double u) const ;
};



class Shooter : public RobotBehavior {
    public:

        double translation_velocity;
        double translation_acceleration;

        double angular_velocity;
        double angular_acceleration;

        double calculus_step;

        Vector2d goal_center;
        double front_size;
        double radius_ball;

        double robot_radius;

        RobotControlWithCurve robot_control;

        Translation_for_shooting shooting_translation;
        Rotation_for_shooting shooting_rotation;
        
        Translation_for_home home_translation;
        Rotation_for_home home_rotation;

    public:
        Shooter(
            const Vector2d & goal_center, double robot_radius,
            double front_size, double radius_ball,
            double translation_velocity,
            double translation_acceleration,
            double angular_velocity,
            double angular_acceleration,
            double calculus_step,
            double time, double dt
        ); 

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit
        );

        virtual void update(
            double time,
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        void go_to_shoot(
            const Vector2d & ball_position, 
            const Vector2d & robot_position,
            double robot_orientation,
            double time, double current_dt
        );
        void go_home(
            const Vector2d & ball_position, 
            const Vector2d & robot_position,
            double robot_orientation,
            double time, double current_dt
        );

        bool is_static() const;

        virtual Control control() const;


};

}; //Namespace Rhoban

#endif
