#ifndef __STRATEGY__PREPARE_KICKOFF__H__
#define __STRATEGY__PREPARE_KICKOFF__H__

#include "Strategy.h"
#include <string>
#include <robot_behavior/robot_behavior.h>

namespace RhobanSSL {
namespace Strategy {


class Prepare_kickoff : public Strategy {
    private:
        bool is_kicking;
    public:

        Prepare_kickoff(Ai::AiData & game_state);
        bool behavior_has_been_assigned;
        int min_robots() const;
        int max_robots() const;

        static const std::string name;

        Robot_behavior::RobotBehavior* create_follower(
            const Vector2d & follower_position,
            const ContinuousAngle& angle,
            double time, double dt
        ) const;

        void start(double time);
        void stop(double time);

        void set_kicking( bool value = true );
        
        void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );
        virtual ~Prepare_kickoff();
}; 

};
};

#endif
