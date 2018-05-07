#ifndef __STRATEGY__HALT__H__
#define __STRATEGY__HALT__H__

#include "Strategy.h"
#include <string>

namespace RhobanSSL {
namespace Strategy {


class Halt : public Strategy {
    public:
        Halt(Ai::AiData & ai_data);

        int min_robots() const;
        int max_robots() const;

        static const std::string name;

        void start(double time);
        void stop(double time);
        
        void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );
        virtual ~Halt();
}; 

};
};

#endif
