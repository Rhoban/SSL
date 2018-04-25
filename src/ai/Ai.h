#ifndef __AI__H__
#define __AI__H__

#include <com/AICommander.h>
#include <vision/AIVisionClient.h>
#include <robot_behavior/robot_behavior.h>
#include "AiData.h"
#include <referee/Referee.h>
#include <core/machine_state.h>
#include <manager/Manager.h>

namespace RhobanSSL
{

    class AI
    {
    public:

        AI(
            Data & data, 
            AICommander *commander
        );

        void run();
        void stop();

    protected:
        bool running;

        Vision::VisionData visionData;
        Ai::AiData game_state;

        bool enable_kicking;

        AICommander *commander;

        std::map<
            int, 
            std::shared_ptr<RobotBehavior>
        > robot_behaviors;
        
        void stop_all_robots();
        void update_robots( );
        double current_time;
        double current_dt;

        Data & data;
        Referee referee;
        Manager::Manager strategy_manager;
        
        Control update_robot( 
            RobotBehavior & robot_behavior,
            double time, Ai::Robot & robot, Ai::Ball & ball
        );

        void prepare_to_send_control( int robot_id, Control control );

        void limits_velocity( Control & ctrl ) const ;
    };
};

#endif
