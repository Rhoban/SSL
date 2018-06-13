#ifndef __VISION__ROBOT_POSITION_FILTER__H__
#define __VISION__ROBOT_POSITION_FILTER__H__

#include <AiData.h>
#include <VisionClient.h>
#include <math/ContinuousAngle.h>
#include <AiData.h>

namespace RhobanSSL {
namespace vision {

class Robot_position_filter {
    public:
    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > average_filter(
        int robot_id, Ai::Team team_color, 
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined
    );

};

};
};
#endif

