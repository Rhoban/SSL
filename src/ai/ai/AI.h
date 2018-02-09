#pragma once

#include <com/AICommander.h>
#include <vision/AIVisionClient.h>
#include "robot_behavior.h"
#include "AiData.h"
#include <referee/Referee.h>

namespace RhobanSSL
{
    class AI
    {
    public:
        AI(Data & data, AICommander *commander);

        void tick();
        void run();
        void stop();

    protected:
        bool running;

        double time_sync;

        Ai::AiData game_state;

        bool enable_kicking;

        AICommander *commander;
        Data & data;
        Goalie goalie;
        Shooter shooter;

        Referee referee;
        
        double max_velocity_t;
        double max_velocity_r;
        double max_acceleration_t;
        double max_acceleration_r;

        Control update_robot( 
            RobotBehavior & robot_behavior,
            double time, Ai::Robot & robot, Ai::Ball & ball
        );

        void prepare_to_send_control( int robot_id, Control control );
    };
};
