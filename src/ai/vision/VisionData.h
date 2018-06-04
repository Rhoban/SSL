#ifndef __VISIONDATA_H__
#define __VISIONDATA_H__

#include <map>
#include <rhoban_geometry/point.h>
#include <math/ContinuousAngle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/MovementSample.h>
#include <iostream>

namespace RhobanSSL {
namespace Vision {

static const int history_size = 3;
static const int Robots = 8;

typedef enum {
    Ally,
    Opponent
} Team;


struct Object {
    MovementSample movement;

    bool present;
    int id;
    rhoban_utils::TimeStamp lastUpdate;

    void update(
        double time, const rhoban_geometry::Point & linear_position,
        const rhoban_utils::Angle & angular_position
    );
    void update(
        double time, const rhoban_geometry::Point & linear_position,
        const ContinuousAngle & angular_position
    );
    void update(
        double time, const rhoban_geometry::Point & linear_position
    );

    double age() const;
    bool isOk() const;

    Object();
    void checkAssert( double time ) const;
};

std::ostream& operator<<(std::ostream& out, const Object& object);

struct Robot : Object { };
struct Ball : Object { };

struct Field
{
    bool present;
    float fieldLength;
    float fieldWidth;
    float goalWidth;
    float goalDepth;
    float boundaryWidth;
    float penaltyAreaDepth;
    float penaltyAreaWidth;
};

class VisionData {
public:
    VisionData();

    std::map<Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;

    double older_time() const;
    void checkAssert( double time ) const;

    void print() const;

    friend std::ostream& operator<<(std::ostream& out, const RhobanSSL::Vision::VisionData& vision);

};

std::ostream& operator<<(std::ostream& out, const VisionData& vision);

} }


#endif
