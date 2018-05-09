#ifndef __pid__H__
#define __pid__H__

#include <debug.h>
#include <math/ContinuousAngle.h>
#include <math/vector2d.h>
#include <utility>

struct PidControl {
    Vector2d velocity_translation;
    ContinuousAngle velocity_rotation;

    PidControl();
    PidControl(
        const Vector2d & velocity_translation,
        ContinuousAngle velocity_rotation
    );
    PidControl relative_control(
        const ContinuousAngle & angular_orientation, double dt
    ) const ;
};

std::ostream& operator << ( std::ostream &, const PidControl& control );

struct PidController {
    double kp_t,ki_t,kd_t;
    double kp_o,ki_o,kd_o;

    bool static_robot;

    double start_time;
    double time;
    double dt;

    void init_time(double start_time, double dt);

    void update(double current_time);
   
    PidController();
    PidController(
        double p_t, double i_t, double d_t, 
        double p_o, double i_o, double d_o 
    );

    void set_orientation_pid( double kp, double ki, double kd );
    void set_translation_pid( double kp, double ki, double kd );

    void set_static(bool value);
    bool is_static() const ;

    double get_dt() const;
    double get_time() const;

    virtual ContinuousAngle goal_orientation( double t ) const =0;
    virtual Vector2d goal_position( double t ) const = 0;

    Vector2d no_limited_translation_control(
        const Vector2d & robot_position, 
        ContinuousAngle robot_orientation
    ) const;
    double no_limited_angular_control(
        const Vector2d & robot_position, 
        ContinuousAngle robot_orientation
    ) const;
    
    virtual PidControl no_limited_control(
        const Vector2d & robot_position, 
        ContinuousAngle robot_orientation
    ) const;
};

#endif
