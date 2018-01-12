#pragma once

#include "AICommander.h"
#include "AIVisionClient.h"

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
    };
}
