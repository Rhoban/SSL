#include "GameState.h"

namespace RhobanSSL
{
    GameState::GameState()
    {
        for (auto team : {Ally, Opponent}) {
            for (int k=0; k<Robots; k++) {
                robots[team][k].present = false;    
            }
        }
    }
}
