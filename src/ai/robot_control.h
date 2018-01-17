#ifndef __MOVMENT__H__
#define __MOVMENT__H__


#include "debug.h"
#include <Eigen/Dense>
#include <vector>

#include "curve.h"
#include "pid.h"

namespace tools {
    struct fct_wrapper {
        std::function<double (double u)> rotation;

        fct_wrapper(
            const std::function<double (double u)> & rotation
        ):rotation(rotation){ };

        Eigen::Vector2d operator()(double t){
            return Eigen::Vector2d( rotation(t), 0.0 );
        };
    };
}

class CurveForRobot {
      public:
        tools::fct_wrapper rotation_fct;

        Curve2d translation_curve;
        Curve2d angular_curve;

        VelocityConsign tranlsation_consign;
        VelocityConsign angular_consign;

        RenormalizedCurve translation_movment;
        RenormalizedCurve rotation_movment;

    public:
        CurveForRobot(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double translation_velocity, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double angular_acceleration, 
            double calculus_step
        );
        Eigen::Vector2d translation(double t) const;
        double rotation(double t) const;

        void print_translation_curve( double dt ) const;
        void print_translation_movment( double dt ) const;
       
        void print_rotation_curve( double dt ) const;
        void print_rotation_movment( double dt ) const;
};


class RobotControlWithCurve : public PidControl {
    public:
        CurveForRobot curve;

        RobotControlWithCurve();
        RobotControlWithCurve( double p, double i, double d );
        RobotControlWithCurve(
            double p_t, double i_t, double d_t, 
            double p_o, double i_o, double d_o 
        );

        void set_movment(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double translation_velocity, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double angular_acceleration, 
            double calculus_step, double current_time
        );

        double goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

class RobotControlWithPositionFollowing : public PidControl{
    protected:
        Eigen::Vector2d position;
        double orientation;

    public:
        void set_goal(
            const Eigen::Vector2d & position, double orientation
        );

        double goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

#endif
