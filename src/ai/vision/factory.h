#ifndef __VISION__FACTORY__H__
#define __VISION__FACTORY__H__

#include "robot_position_filter.h"

namespace RhobanSSL {
namespace vision {

class Factory {
    public:
    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > filter(
        int robot_id, Ai::Team team_color, 
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined
    );
};

};
};
#endif
