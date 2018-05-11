#ifndef __AIDATA_H__
#define __AIDATA_H__

#include <debug.h>
#include <map>
#include <list>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/MovementSample.h>
#include <vision/VisionData.h>
#include <physic/Movement.h>
#include <math/frame_changement.h>

// Comment the following line if you are working with the real robot.
// If you are working with the grSim simulator, don't comment.
#define SSL_SIMU

namespace RhobanSSL {
namespace Ai {

typedef enum {
    Yellow, Blue, Unknown
} Team;

class Object {
private: 
    Vision::Object vision_data;
    RhobanSSL::Movement * movement;


public:

    int id() const {
        return vision_data.id;
    }

    Object();
    Object( const Object& object );
    Object& operator=( const Object& object );

    void set_vision_data( const Vision::Object & vision_data  );
    void set_movement( Movement * movement );

    //We assume that v1 and v2 are orthonormal
    void change_frame(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    );

    bool isOk() const ;

    const RhobanSSL::Movement & get_movement() const; 
    virtual ~Object();
};

class Robot : public Object { };
class Ball : public Object { };

struct Field : Vision::Field {

    bool is_inside( const rhoban_geometry::Point & point ) const {
        return (
            std::fabs( point.getX() ) < (fieldLength/2.0 + boundaryWidth)
            and
            std::fabs( point.getY() ) < (fieldWidth/2.0 + boundaryWidth)
        );
    }
};

struct Constants {

    static constexpr double WHEEL_RADIUS = 0.03; // In meter
    static constexpr double WHEEL_EXCENTRICITY = 0.08; // In meter
    static constexpr double WHEEL_NB_TURNS_ACCELERATION_LIMIT = 70.0; // TODO : SET TO 10 turn by second
    static constexpr double TRANSLATION_ACCELERATION_LIMIT = WHEEL_NB_TURNS_ACCELERATION_LIMIT*WHEEL_RADIUS*2.0*M_PI; // 10 tours * 0.03 m * 2*PI  m/s^-2
    static constexpr double ROTATION_ACCELERATION_LIMIT = 2.0*M_PI*(WHEEL_RADIUS/WHEEL_EXCENTRICITY)*WHEEL_NB_TURNS_ACCELERATION_LIMIT; //

#ifdef SSL_SIMU
    static constexpr double ROTATION_VELOCITY_LIMIT = 3.0;
    static constexpr double TRANSLATION_VELOCITY_LIMIT = 8.0;
#else
#endif

    double robot_radius;
    double radius_ball;
    double front_size;
    Vector2d left_post_position;
    Vector2d right_post_position;
    Vector2d goal_center;
    Vector2d waiting_goal_position;
    // PID for translation
    double p_translation;
    double i_translation;
    double d_translation;
    // PID for orientation
    double p_orientation;
    double i_orientation;
    double d_orientation;

    double translation_velocity;
    double translation_acceleration;
    double angular_velocity;
    double angular_acceleration;

    double calculus_step;
    bool enable_kicking;

    double penalty_rayon;
    double translation_velocity_limit;
    double rotation_velocity_limit;
    double translation_acceleration_limit;
    double rotation_acceleration_limit;

    double time_limit_between_collision; 
    double security_acceleration_ratio;
    double radius_security_for_collision;    

    void init();

    Constants(){ 
        init();
    }

};

class AiData {
public:
    double time_shift_with_vision;
    double time; //(Write for Ai)
    double dt; //(Write for Ai)

    std::string team_name;
    Ai::Team team_color;

    AiData();


    typedef std::map<int, Robot> Robots_table;
    typedef std::map<Vision::Team, Robots_table> Robots_table_by_team;
    Robots_table_by_team robots;

    std::vector< std::pair<Vision::Team, Robot*> > all_robots;
    
    Ball ball;
    Field field;

    // the key is a pair of robot identifeds.
    typedef std::map< std::pair<int, int>, double > Collision_times_table;

    Collision_times_table table_of_collision_times;


    Frame_changement team_point_of_view;

    void change_frame_for_all_objects(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    );
    void change_team_color( Ai::Team team_color );

    Constants constants;

    void update( const Vision::VisionData vision_data);

    // Rturn true is the robot is ready and inside the field
    bool robot_is_valid( int robot_id ) const;
    bool robot_is_inside_the_field( int robot_id ) const;

    void visit_all_pair_of_robots(
        std::function <
            void (
                Vision::Team robot_team_1, Robot & robot_1,
                Vision::Team robot_team_2, Robot & robot_2 
            )
        > visitor
    );

    const Collision_times_table & get_table_of_collision_times() const;

    void compute_table_of_collision_times();
    std::list< std::pair<int, double> > get_collision( int robot_id, const Vector2d & ctrl ) const;

};

} }

#endif
