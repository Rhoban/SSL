#ifndef __MOVMENT__H__
#define __MOVMENT__H__


#include <debug.h>
#include <Eigen/Dense>
#include <vector>

#include <math/curve.h>
#include "pid.h"


namespace robot_control_details {
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
        robot_control_details::fct_wrapper rotation_fct;

        Curve2d translation_curve;
        Curve2d angular_curve;

        //DifferentiableVelocityConsign tranlsation_consign;
        //DifferentiableVelocityConsign angular_consign;
        ContinuousVelocityConsign tranlsation_consign;
        ContinuousVelocityConsign angular_consign;

        double calculus_error_for_translation;
        double calculus_error_for_rotation;

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


class RobotControl {
    private:
        double translation_velocity_limit;
        ContinuousAngle rotation_velocity_limit;

        PidControl limited_control(
            const Eigen::Vector2d & robot_position, 
            ContinuousAngle robot_orientation
        ) const;

    public:
        RobotControl(): 
            translation_velocity_limit(-1),
            rotation_velocity_limit(-1)
        { };

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit
        );

        virtual PidControl absolute_control_in_absolute_frame(
            const Eigen::Vector2d & robot_position, 
            ContinuousAngle robot_orientation
        ) const = 0;

        virtual double get_dt() const = 0;

        PidControl absolute_control_in_robot_frame(
            const Eigen::Vector2d & robot_position, 
            ContinuousAngle robot_orientation
        ) const;

        PidControl relative_control_in_robot_frame(
            const Eigen::Vector2d & robot_position, 
            ContinuousAngle robot_orientation
        ) const;

};

class RobotControlWithPid : public RobotControl, public PidController {
    public:
        PidControl absolute_control_in_absolute_frame(
            const Eigen::Vector2d & robot_position, 
            ContinuousAngle robot_orientation
        ) const;

        double get_dt() const;
};


class RobotControlWithCurve : public RobotControlWithPid {
    public:
        CurveForRobot curve;

        RobotControlWithCurve();

        void set_movement(
            const std::function<Eigen::Vector2d (double u)> & translation,
            double translation_velocity, double translation_acceleration,
            const std::function<double (double u)> & rotation,
            double angular_velocity, double angular_acceleration, 
            double calculus_step, double current_time
        );

        ContinuousAngle goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

class RobotControlWithPositionFollowing : public RobotControlWithPid {
    protected:
        Eigen::Vector2d position;
        ContinuousAngle orientation;

    public:
        void set_goal(
            const Eigen::Vector2d & position, ContinuousAngle orientation
        );

        ContinuousAngle goal_orientation( double t ) const;
        Eigen::Vector2d goal_position( double t ) const;
};

#endif
