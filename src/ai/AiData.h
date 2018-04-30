#ifndef __AIDATA_H__
#define __AIDATA_H__

#include <map>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/MovementSample.h>
#include <vision/VisionData.h>
#include <physic/Movement.h>
#include <math/frame_changement.h>
#include <Eigen/Dense>

// Comment the following line if you are working with the real robot.
// If you are working with the grSim simulator, don't comment.
#define SSL_SIMU

#ifdef SSL_SIMU
    #define ROTATION_VELOCITY_LIMIT -1.0
    #define TRANSLATION_VELOCITY_LIMIT -1.0
#else
    //#define ROTATION_VELOCITY_LIMIT 3.0
    //#define TRANSLATION_VELOCITY_LIMIT 2.0
    #define ROTATION_VELOCITY_LIMIT 20.0
    #define TRANSLATION_VELOCITY_LIMIT 8.0
#endif

namespace RhobanSSL {
namespace Ai {

typedef enum {
    Yellow, Blue
} Team;

class Object {
private: 
    Vision::Object vision_data;
    RhobanSSL::Movement * movement;


public:

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

struct Field : Vision::Field { };

struct Constants {
    double robot_radius;
    double radius_ball;
    double front_size;
    Eigen::Vector2d left_post_position;
    Eigen::Vector2d right_post_position;
    Eigen::Vector2d goal_center;
    Eigen::Vector2d waiting_goal_position;
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
    
    void init();

    Constants(){ 
        init();
    }

};

class AiData {
public:

    AiData();

    std::map<Vision::Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;

    Frame_changement team_point_of_view;

    void change_frame_for_all_objects(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    );

    Constants constants;

    void update( const Vision::VisionData vision_data);
};

} }

#endif
