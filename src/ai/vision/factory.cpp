#include "factory.h"
    
namespace RhobanSSL {
namespace vision {

std::pair<
    rhoban_geometry::Point,
    ContinuousAngle
>
Factory::filter(
    int robot_id, Ai::Team team_color, 
    const std::map<int, SSL_DetectionFrame> & camera_detections,
    bool & orientation_is_defined
){
    return Robot_position_filter::average_filter(
        robot_id, team_color, camera_detections, orientation_is_defined
    );
}

}
}
