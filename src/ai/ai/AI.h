#pragma once

#include <com/AICommander.h>
#include <vision/AIVisionClient.h>
#include "robot_behavior.h"
#include "AiData.h"
#include <referee/Referee.h>
#include <core/machine_state.h>

namespace RhobanSSL
{
    class TeamId {
        public:
        static const int goalie_id; 
        static const int shooter_id;
        static const int follower_id; 
    };

    class AI
    {
    public:

        AI(
            Data & data, 
            AICommander *commander_yellow,
            AICommander *commander_blue
        );

        void run();
        void stop();

    protected:
        typedef construct_machine_state_infrastructure<
            std::string, Ai::AiData, Ai::AiData
        > machine_infrastructure;

        bool running;

        Vision::VisionData visionData;
        Ai::AiData game_state;
        Ai::Constants & constants;

        machine_infrastructure::MachineState machine;

        bool enable_kicking;

        AICommander *commander_yellow;
        AICommander *commander_blue;


        std::map<
            Vision::Team, std::map<
                int, 
                std::shared_ptr<RobotBehavior>
            >
        > robot_behaviors;
        bool time_synchro;
        double waiting_time_for_synchro;
        double start_waiting_time_for_synchro;
        
        void stop_all_robots();
        void assign_behavior_to_robots();
        void update_robots( );
        void try_to_synchronize_time();
        bool time_is_synchronized() const;
        double current_time;
        double current_dt;

        Data & data;
        Referee referee;
        
        double max_velocity_t;
        double max_velocity_r;
        double max_acceleration_t;
        double max_acceleration_r;

        Control update_robot( 
            RobotBehavior & robot_behavior,
            double time, Ai::Robot & robot, Ai::Ball & ball
        );

        void prepare_to_send_control( 
            Vision::Team team, int robot_id, Control control
        );

        void limits_velocity( Control & ctrl ) const ;

    };
};
