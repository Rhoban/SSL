#include "GameState.h"



namespace RhobanSSL
{
    double GameState::Ball::age()
    {
        return diffSec(lastUpdate, Utils::Timing::TimeStamp::now());
    }

    bool GameState::Ball::isOk()
    {
        return present && age() < 1;
    }

    double GameState::Robot::age()
    {
        return diffSec(lastUpdate, Utils::Timing::TimeStamp::now());
    }

    bool GameState::Robot::isOk()
    {
        return present && age() < .2;
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
