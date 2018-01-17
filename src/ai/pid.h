#ifndef __pid__H__
#define __pid__H__

#include "debug.h"
#include <Eigen/Dense>

#define ROTATION_VELOCITY_LIMIT 3.0
#define TRANSLATION_VELOCITY_LIMIT 2.0
//#define ROTATION_VELOCITY_LIMIT 20.0
//#define TRANSLATION_VELOCITY_LIMIT 5.0


struct Control {
    Eigen::Vector2d velocity_translation;
    double velocity_rotation;

/*
    void limits_the_contol(
        double rotation_velocity_limit, // 0.0 means no limit
        double translation_velocity_limit // 0.0 means no limit
    );
*/
};

struct PidControl {
    double kp_t,ki_t,kd_t;
    double kp_o,ki_o,kd_o;

    bool static_robot;

    double start_time;
    double time;
    double dt;

    void init_time(double start_time);

    void update(double current_time);
   
    PidControl();
    PidControl( double p, double i, double d );
    PidControl(
        double p_t, double i_t, double d_t, 
        double p_o, double i_o, double d_o 
    );

    void set_pid( double kp, double ki, double kd );
    void set_orientation_pid( double kp, double ki, double kd );
    void set_translation_pid( double kp, double ki, double kd );

    void set_static(bool value);
    bool is_static() const ;

    virtual double goal_orientation( double t ) const =0;
    virtual Eigen::Vector2d goal_position( double t ) const = 0;

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
};

#endif
