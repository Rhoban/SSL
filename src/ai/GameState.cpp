#include "GameState.h"

namespace RhobanSSL
{
    double GameState::Ball::age()
    {
        Utils::Timing::TimeStamp now;
        return diffSec(lastUpdate, now);
    }

    bool GameState::Ball::isOk()
    {
        return present && age() < 1;
    }

    double GameState::Robot::age()
    {
        Utils::Timing::TimeStamp now;
        return diffSec(lastUpdate, now);
    }

    bool GameState::Robot::isOk()
    {
        return present && age() < 1;
    }

    GameState::GameState()
    {
        field.present = false;

        for (auto team : {Ally, Opponent}) {
            for (int k=0; k<Robots; k++) {
                robots[team][k].present = false;
            }
        }
    }
}
