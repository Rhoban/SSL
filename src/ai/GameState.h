#pragma once

#include <map>
#include <geometry/Point.hpp>
#include <geometry/Angle.hpp>
#include <timing/TimeStamp.hpp>

namespace RhobanSSL
{
class GameState
{
public:
    static const int Robots = 6;

    typedef enum {
        Ally,
        Opponent
    } Team;
    struct Robot
    {
        Utils::Timing::TimeStamp lastUpdate;
        bool present;
        int id;
        Point position;
        Angle orientation;

        double age();
        bool isOk();
    };
    struct Ball
    {
        Utils::Timing::TimeStamp lastUpdate;
        bool present;
        Point position;

        double age();
        bool isOk();
    };
    struct Field
    {
        bool present;
        float fieldLength;
        float fieldWidth;
        float goalWidth;
    };

    GameState();

    std::map<Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;
};
}
