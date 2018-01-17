#include "robot_control.h"
#include <geometry/Angle.hpp>

class Goalie {
    private:
        Eigen::Vector2d left_post_position; 
        Eigen::Vector2d right_post_position;
        Eigen::Vector2d goal_center;
        Eigen::Vector2d waiting_goal_position;

        Eigen::Vector2d robot_position;
        double robot_orientation;

        RobotControlWithPositionFollowing robot_control;
 
        double goalie_radius = .1;


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

        void init(
            const Eigen::Vector2d & left_post_position,
            const Eigen::Vector2d & right_post_position,
            const Eigen::Vector2d & waiting_goal_position,
            double goalie_radius
        );

        void update(
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time
        );

        Control control() const;
};



struct Translation_for_shooting {
    Eigen::Vector2d position_robot;
    Eigen::Vector2d position_ball;
    Eigen::Vector2d goal_center;
    
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



class Shooter {
    private:

        // BUG : SI vous obtenez une boucle infini, vous devez bidoullier les paramètres (en baissant l'acceleration).
        //  Le bug sera résolu plus tard.
        double translation_velocity = 0.8;
        double translation_acceleration = 0.4;

        double angular_velocity = 1.5;  
        double angular_acceleration = 0.7;

        double calculus_step = 0.0001;

        Eigen::Vector2d goal_center;

        Eigen::Vector2d robot_position;
        double robot_orientation;
        Eigen::Vector2d ball_position;

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

        void init(
            const Eigen::Vector2d & goal_center, double robot_radius
        );

        void update(
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time
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


