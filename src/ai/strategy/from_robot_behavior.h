#ifndef __STRATEGY__FROM_ROBOT_BEHAVIOR__H__
#define __STRATEGY__FROM_ROBOT_BEHAVIOR__H__

#include "Strategy.h"
#include <string>
#include <memory>

namespace RhobanSSL {
namespace Strategy {


class From_robot_behavior: public Strategy {
    private:
        std::function<
            std::shared_ptr<Robot_behavior::RobotBehavior> (double time, double dt) 
        > robot_behavior_allocator;
        bool behavior_has_been_assigned;
        bool is_goalie;

    public:

        From_robot_behavior( 
            std::function<
                std::shared_ptr<Robot_behavior::RobotBehavior>(double time, double dt)
            > robot_behavior_allocator,
            bool is_goalie = false
        );
        virtual int min_robots() const;
        virtual int max_robots() const;

        static const std::string name;

        virtual void start(double time);
        virtual void stop(double time);
        
        virtual void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );

        virtual ~From_robot_behavior();
}; 


};
};

#endif
