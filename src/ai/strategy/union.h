#ifndef __STRATEGY__UNION__H__
#define __STRATEGY__UNION__H__

#include "Strategy.h"
#include <string>
#include <memory>

namespace RhobanSSL {
namespace Strategy {


class Union : public Strategy {
    private:
    std::list<std::shared_ptr<Strategy>> strategies_without_goal;
    std::shared_ptr<Strategy> strategy_with_goal;
    int min;
    int max;
 
    public:

    Union();

    void  clear();

    void add_goalie_strategy( std::shared_ptr<Strategy> strategy );
    void add_strategy( std::shared_ptr<Strategy> strategy );

    virtual void update(double time);

    virtual void start(double time);
    virtual void stop(double time);
    virtual void pause(double time);
    virtual void resume(double time);

    virtual int min_robots() const;
    virtual int max_robots() const;

    virtual void assign_behavior_to_robots(
        std::function<
            void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
        > assign_behavior,
        double time, double dt
    );

    virtual ~Union();
};

};
};
#endif
