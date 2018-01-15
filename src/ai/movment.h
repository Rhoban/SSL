#ifndef __MOVMENT__H__
#define __MOVMENT__H__


#include "debug.h"
#include <Eigen/Dense>
#include <vector>

struct VelocityConsign {
    double distance;
    double max_acceleration; 
    double max_velocity;

    VelocityConsign(
        double distance, double max_acceleration, 
        double max_velocity
    );

    double operator()(double t);
    double time_of_deplacement();
    double time_of_acceleration();
};

class Curve2d {
//    protected:
    public:
        std::function<Eigen::Vector2d (double u)> curve;
        
        double step_time;
        double curve_length;
        double time_max;

        void init();

    public:
        Curve2d(
            const std::function<Eigen::Vector2d (double u)> & curve,
            double step_time
        );
        Curve2d( const Curve2d& curve );

        Eigen::Vector2d operator()(double u) const;

        double arc_length( double u ) const;
        double inverse_of_arc_length( double l ) const;
        double size() const;
        
};

class RenormalizedCurve : public Curve2d {
//    protected:
    public:
        std::function<double (double t)> velocity_consign;
        double time_max;

        void init();

    public:
        RenormalizedCurve(
            const std::function<Eigen::Vector2d (double u)> & curve,
            const std::function<double (double t)> & velocity_consign,
            double step_time
        );
        RenormalizedCurve(
            const Curve2d & curve,
            const std::function<double (double t)> & velocity_consign
        );

        void set_step_time( double dt );

        double max_time() const;
        Eigen::Vector2d original_curve( double u ) const;
        double time( double length ) const;
        double position_consign( double t ) const;
        Eigen::Vector2d operator()(double t) const;

        double error_position_consign() const;
        double get_step_time() const;
};

struct fct_wrapper {
    std::function<double (double u)> rotation;

    fct_wrapper(
        const std::function<double (double u)> & rotation
    ):rotation(rotation){ };

    Eigen::Vector2d operator()(double t){
        return Eigen::Vector2d( rotation(t), 0.0 );
    };
};

class CurveForRobot {
//    protected:
      public:
        fct_wrapper rotation_fct;

        Curve2d translation_curve;
        Curve2d angular_curve;

        VelocityConsign tranlsation_consign;
        VelocityConsign angular_consign;

        RenormalizedCurve translation_movment;
        RenormalizedCurve rotation_movment;

    public:
        CurveForRobot(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double angular_acceleration, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double translation_velocity,
            double calculus_step
        );
        Eigen::Vector2d translation(double t);
        double rotation(double t);

        void print_translation_curve( double dt ) const;
        void print_translation_movment( double dt ) const;
       
        void print_rotation_curve( double dt ) const;
        void print_rotation_movment( double dt ) const;
};

struct Control {
    Eigen::Vector2d velocity_translation;
    double velocity_rotation;
};

class RobotControl {
//    protected:
    public:
        CurveForRobot curve;

        double kp_t,ki_t,kd_t;
        double kp_o,ki_o,kd_o;
        bool static_robot;
    public:
        double start_time;
        double time;
        double dt;
    public:
        RobotControl();
        RobotControl( double p, double i, double d );
        RobotControl(
            double p_t, double i_t, double d_t, 
            double p_o, double i_o, double d_o 
        );
        void update(double current_time);
        void set_static();
        bool is_static() const ;
        void set_movment(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double angular_acceleration, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double translation_velocity,
            double calculus_step, double current_time
        );

        Eigen::Vector2d translation_control_in_absolute_frame(
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );
        double rotation_control_in_absolute_frame(
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );

        Control relative_control_in_robot_frame(
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );

        Control absolute_control_in_robot_frame(
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );

        double relative_command();

        void set_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );
        void set_translation_pid( double kp, double ki, double kd );
};


#endif
