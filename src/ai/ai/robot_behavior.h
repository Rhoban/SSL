#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <geometry/Angle.hpp>
#include "Data.h"
#include "AiData.h"

namespace RhobanSSL
{

struct Control : PidControl {
    bool kick;
    bool active;
    bool ignore;

    Control();
    Control(bool kick, bool active, bool ignore);
    Control(const PidControl& c);

    static Control make_desactived();
    static Control make_ignored();
    static Control make_null();
};



class RobotBehavior {
    protected:
    
        double birthday;
        double lastUpdate; 

        Eigen::Vector2d robot_position;
        double robot_orientation;
        Eigen::Vector2d ball_position;

    public:
        RobotBehavior();
        
        double age() const;
        bool is_born() const;
        double set_birthday( double birthday );

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );
        virtual Control control() const = 0;
};


class Goalie : public RobotBehavior {
    private:
        Eigen::Vector2d left_post_position; 
        Eigen::Vector2d right_post_position;
        Eigen::Vector2d goal_center;
        Eigen::Vector2d waiting_goal_position;

        RobotControlWithPositionFollowing robot_control;
 
        double goalie_radius;
        double penalty_rayon;

        static Eigen::Vector2d calculate_goal_position(
            const Eigen::Vector2d & ball_position,
            const Eigen::Vector2d & poteau_droit,
            const Eigen::Vector2d & poteau_gauche,
            double goalie_radius
        );

    public:
        Goalie(); 

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit
        );

        void init(
            const Eigen::Vector2d & left_post_position,
            const Eigen::Vector2d & right_post_position,
            const Eigen::Vector2d & waiting_goal_position,
            double penalty_rayon,
            double goalie_radius
        );

        void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        Control control() const;
};



struct Translation_for_shooting {
    Eigen::Vector2d position_robot;
    Eigen::Vector2d position_ball;
    Eigen::Vector2d goal_center;
   
    double front_size;
    double radius_ball;
 
    Eigen::Vector2d operator()(double u) const;
};

struct Rotation_for_shooting {
    double orientation;
    double end;

    double operator()(double u) const;
};

struct Translation_for_home {
    Eigen::Vector2d position_robot;
    Eigen::Vector2d position_home;
    
    Eigen::Vector2d operator()(double u) const;
};

struct Rotation_for_home {
    double orientation;
    Eigen::Vector2d position_ball;
    Eigen::Vector2d position_robot;

    double operator()(double u) const ;
};



class Shooter : public RobotBehavior {
    public:

        double translation_velocity;
        double translation_acceleration;

        double angular_velocity;
        double angular_acceleration;

        double calculus_step;

        Eigen::Vector2d goal_center;
        double front_size;
        double radius_ball;

        double robot_radius;

        RobotControlWithCurve robot_control;

        Translation_for_shooting shooting_translation;
        Rotation_for_shooting shooting_rotation;
        
        Translation_for_home home_translation;
        Rotation_for_home home_rotation;

    public:
        Shooter(); 

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit
        );

        void init(
            const Eigen::Vector2d & goal_center, double robot_radius,
            double front_size, double radius_ball,
            double translation_velocity,
            double translation_acceleration,
            double angular_velocity,
            double angular_acceleration,
            double calculus_step
        );

        void update(
            double time,
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        void go_to_shoot(
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time
        );
        void go_home(
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time
        );

        bool is_static() const;

        Control control() const;


};

}; //Namespace Rhoban
