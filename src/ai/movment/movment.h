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
    protected:
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
    protected:
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

class CurveForRobot {
    protected:
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
};

class RobotControl {
    protected:
        CurveForRobot curve;

        double kp_t,ki_t,kd_t;
        double kp_o,ki_o,kd_o;

    public:
        RobotControl(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double angular_acceleration, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double translation_velocity,
            double calculus_step
        );

        Eigen::Vector2d translation_command(
            double t, double dt,
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );
        double rotation_command(
            double t, double dt,
            const Eigen::Vector2d & robot_position, 
            double robot_orientation
        );
        void set_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );
        void set_tranlsation_pid( double kp, double ki, double kd );
};


#endif
