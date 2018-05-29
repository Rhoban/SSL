#include "factory.h"

#include "position_follower.h"
#include "navigation_with_obstacle_avoidance.h"
#include "a_star_path.h"

namespace RhobanSSL {
namespace Robot_behavior {

ConsignFollower* Factory::fixed_consign_follower(
    Ai::AiData & ai_data,
    const rhoban_geometry::Point & position,
    const ContinuousAngle & angle,
    bool ignore_the_ball
){
    //A_star_path* follower = new A_star_path(ai_data, ai_data.time, ai_data.dt); 
    Navigation_with_obstacle_avoidance* follower = new Navigation_with_obstacle_avoidance(ai_data, ai_data.time, ai_data.dt); 
    // PositionFollower* follower = new PositionFollower(ai_data, ai_data.time, ai_data.dt); 
    follower->set_translation_pid(
        ai_data.constants.p_translation,
        ai_data.constants.i_translation, 
        ai_data.constants.d_translation
    );
    follower->set_orientation_pid(
        ai_data.constants.p_orientation, ai_data.constants.i_orientation, 
        ai_data.constants.d_orientation
    );
    follower->set_limits(
        ai_data.constants.translation_velocity_limit,
        ai_data.constants.rotation_velocity_limit,
        ai_data.constants.translation_acceleration_limit,
        ai_data.constants.rotation_acceleration_limit
    );
    follower->set_following_position(position, angle);
    follower->avoid_the_ball(not(ignore_the_ball));
    return follower;
}

};
};
