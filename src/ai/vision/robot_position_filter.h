#ifndef __VISION__ROBOT_POSITION_FILTER__H__
#define __VISION__ROBOT_POSITION_FILTER__H__

#include <AiData.h>
#include <VisionClient.h>
#include <math/ContinuousAngle.h>
#include <AiData.h>
#include "VisionData.h"

namespace RhobanSSL {
namespace Vision {

enum Part_of_the_field {
    POSIVE_HALF_FIELD,
    NEGATIVE_HALF_FIELD,
    ALL_FIELD
};

bool object_coordonate_is_valid(
    double x,double y,
    Part_of_the_field part_of_the_field_used
);


class Robot_position_filter {
    public:
    static std::pair<
        rhoban_geometry::Point,
        ContinuousAngle
    > average_filter(
        int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally, 
        const std::map<int, SSL_DetectionFrame> & camera_detections,
        bool & orientation_is_defined, const Vision::VisionData & old_vision_data,
        Vision::Part_of_the_field part_of_the_field_used
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

