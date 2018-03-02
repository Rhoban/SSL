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

std::ostream& operator << ( std::ostream &, const Control& control  );

class RobotBehavior {
    protected:
    
        double birthday;
        double lastUpdate; 

        Eigen::Vector2d robot_position;
        ContinuousAngle robot_orientation;
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


class DoNothing : public RobotBehavior {
    public:
        DoNothing(); 

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;
};


class PositionFollower : public RobotBehavior {
    private:
        Eigen::Vector2d position;
        ContinuousAngle angle;

        RobotControlWithPositionFollowing robot_control;

    public:
        PositionFollower( double time, double dt ); 

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit
        );

        void set_following_position(
            const Eigen::Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;
};





class Goalie : public PositionFollower {
    private:
        PositionFollower follower();

        Eigen::Vector2d left_post_position; 
        Eigen::Vector2d right_post_position;
        Eigen::Vector2d goal_center;
        Eigen::Vector2d waiting_goal_position;

        double goalie_radius;
        double penalty_rayon;

        static Eigen::Vector2d calculate_goal_position(
            const Eigen::Vector2d & ball_position,
            const Eigen::Vector2d & poteau_droit,
            const Eigen::Vector2d & poteau_gauche,
            double goalie_radius
        );

    public:
        Goalie(
            const Eigen::Vector2d & left_post_position,
            const Eigen::Vector2d & right_post_position,
            const Eigen::Vector2d & waiting_goal_position,
            double penalty_rayon,
            double goalie_radius,
            double time, double dt
        );

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );
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
        Shooter(
            const Eigen::Vector2d & goal_center, double robot_radius,
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
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time, double current_dt
        );
        void go_home(
            const Eigen::Vector2d & ball_position, 
            const Eigen::Vector2d & robot_position,
            double robot_orientation,
            double time, double current_dt
        );

        bool is_static() const;

        virtual Control control() const;


};

}; //Namespace Rhoban
