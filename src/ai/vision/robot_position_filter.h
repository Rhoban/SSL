#ifndef __VISION__ROBOT_POSITION_FILTER__H__
#define __VISION__ROBOT_POSITION_FILTER__H__

#include <AiData.h>
#include <VisionClient.h>
#include <math/ContinuousAngle.h>
#include <AiData.h>
#include "VisionData.h"

namespace RhobanSSL {
namespace Vision {

class Robot_position_filter {
    public:
    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > average_filter(
        int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally, 
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined, const Vision::VisionData & old_vision_data
    );

    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > exponential_degression_filter(
        int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally,
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined, const Vision::VisionData & old_vision_data
    );

    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > no_filter(
        int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally,
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined, const Vision::VisionData & old_vision_data
    );

};

};
};
#endif

