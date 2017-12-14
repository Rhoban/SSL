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
        bool present;
        int id;
        Utils::Timing::TimeStamp lastUpdate;
        Point position;
        Angle orientation;
    };
    struct Ball
    {
        bool present;
        Point position;
    };

    GameState();

protected:
    std::map<Team, std::map<int, Robot>> robots;
    Ball ball;
};
}
