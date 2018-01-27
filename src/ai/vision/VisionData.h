#ifndef __VISIONDATA_H__
#define __VISIONDATA_H__

#include <map>
#include <geometry/Point.hpp>
#include <geometry/Angle.hpp>
#include <timing/TimeStamp.hpp>
#include <tools/MovementSample.h>

namespace RhobanSSL {
namespace Vision {

static const int history_size = 3;
static const int Robots = 9;

typedef enum {
    Ally,
    Opponent
} Team;


struct Object {
    MovementSample movement;

    bool present;
    int id;
    Utils::Timing::TimeStamp lastUpdate;

    void update(
        double time, const Point & linear_position, const Angle & angular_position
    );
    void update(
        double time, const Point & linear_position
    );

    double age() const;
    bool isOk() const;

    Object();
    void checkAssert( double time ) const;
};

struct Robot : Object { };
struct Ball : Object { };

struct Field
{
    bool present;
    float fieldLength;
    float fieldWidth;
    float goalWidth;
};


class VisionData {
public:
    VisionData();

    std::map<Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;

    double older_time() const;
    void checkAssert( double time ) const;
};

} }

#endif
