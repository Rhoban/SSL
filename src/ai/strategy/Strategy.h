#ifndef __STRATEGY__STRATEGY__H__
#define __STRATEGY__STRATEGY__H__

#include <robot_behavior/robot_behavior.h>
#include <map>
#include <memory>

namespace RhobanSSL {
namespace Strategy {

class Strategy {
    public:

    virtual void update(double time){};

    virtual void start(double time){};
    virtual void stop(double time){};
    virtual void pause(double time){};
    virtual void resume(double time){};

    virtual void assign_behavior_to_robots(
        std::map<
            int, 
            std::shared_ptr<RobotBehavior>
        > & robot_behaviors,
        double time, double dt
    ) = 0;

    virtual ~Strategy();
};

};
};
#endif
