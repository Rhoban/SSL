#pragma once

#include "AICommander.h"
#include "AIVisionClient.h"
#include "robot_behavior.h"

namespace RhobanSSL
{

    class AI
    {
    public:
        AI(AIVisionClient *vision, AICommander *commander);

        void tick();
        void run();
        void stop();

    protected:
        bool running;
        AICommander *commander;
        AIVisionClient *vision;
        Goalie goalie;
        Shooter shooter;

        Control update_goalie(
            double time, GameState::Robot & robot, GameState::Ball & ball
        );
        Control update_shooter(
            double time, GameState::Robot & robot, GameState::Ball & ball
        );
        void prepare_to_send_control( int robot_id, Control control );
    };
};
