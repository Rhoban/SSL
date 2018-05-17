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
    private:
        std::string team_name;
        Ai::Team default_team;
    public:

        AI(
            std::string team_name,
            Ai::Team default_team,
            Data & data,
            AICommander *commander
        );

        void run();
        void stop();

        Referee &getReferee();

    protected:
        bool running;

        Vision::VisionData visionData;
        Ai::AiData ai_data;

        bool enable_kicking;

        AICommander *commander;

        std::map<
            int,
            std::shared_ptr<Robot_behavior::RobotBehavior>
        > robot_behaviors;

        void init_robot_behaviors();
        void update_robots( );
        double current_time;
        double current_dt;

        Shared_data shared_data;

        Data & data;
        Referee referee;
        std::shared_ptr<Manager::Manager> strategy_manager;

        Control update_robot(
            Robot_behavior::RobotBehavior & robot_behavior,
            double time, Ai::Robot & robot, Ai::Ball & ball
        );

        void send_control( int robot_id, const Control & control );
        void prepare_to_send_control( int robot_id, Control & control );

        void limits_velocity( Control & ctrl ) const ;
        void check_time_is_coherent() const;

        void share_data();
        void prevent_collision( int robot_id, Control & ctrl );

    };
};

#endif
