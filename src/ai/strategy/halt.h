#ifndef __STRATEGY__HALT__H__
#define __STRATEGY__HALT__H__

#include "Strategy.h"
#include <string>

namespace RhobanSSL {
namespace Strategy {


class Halt : public Strategy {
    public:
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
        virtual ~Halt();
}; 

};
};

#endif
