#ifndef __STRATEGY__SANDBOX__H__
#define __STRATEGY__SANDBOX__H__

#include "Strategy.h"
#include <string>
#include <AiData.h>

namespace RhobanSSL {
namespace Strategy {


class TeamId {
    public:
    static const int goalie_id; 
    static const int shooter_id;
    static const int follower_id; 
};


class Sandbox : public Strategy {
    private:
        Ai::AiData & game_state;
        bool behavior_has_been_assigned;
    public:
        Sandbox(Ai::AiData & game_state);

        static const std::string name;

        void start(double time);
        void stop(double time);
        
        void assign_behavior_to_robots(
            std::map<
                int, 
                std::shared_ptr<RobotBehavior>
            > & robot_behaviors,
            double time, double dt
        );
        virtual ~Sandbox();
}; 

};
};

#endif
