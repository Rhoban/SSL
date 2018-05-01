#include "union.h"

namespace RhobanSSL {
namespace Strategy {

void Union::add_goalie_strategy( std::shared_ptr<Strategy> strategy ){
    min += strategy->min_robots();
    strategy_with_goal = strategy;
}

void Union::add_strategy( std::shared_ptr<Strategy> strategy ){
    if( strategy->max_robots() < 0 ){
        max = -1;
    }else{
        if( max >= 0 ){
            max += strategy->max_robots();
        }
    }
    strategies_without_goal.push_back( strategy );
}

int Union::min_robots() const {
    return min;
}
int Union::max_robots() const {
    return max;
}

Union::~Union(){
}

Union::Union():
    min(0), max(0)
{
}

void Union::clear(){
    strategies_without_goal.clear();
    strategy_with_goal.reset();
    min = 0;
    max = 0;
}

void Union::update(double time){
    if( strategy_with_goal ){
        strategy_with_goal->update(time);
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->update(time);
    }
}

void Union::start(double time){
    if( strategy_with_goal ){
        strategy_with_goal->start(time);
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->start(time);
    }
}

void Union::stop(double time){
    if( strategy_with_goal ){
        strategy_with_goal->stop(time);
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->stop(time);
    }
}

void Union::pause(double time){
    if( strategy_with_goal ){
        strategy_with_goal->pause(time);
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->pause(time);
    }
}

void Union::resume(double time){
    if( strategy_with_goal ){
        strategy_with_goal->resume(time);
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->resume(time);
    }
}


void Union::assign_behavior_to_robots(
    std::function<
        void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
    > assign_behavior,
    double time, double dt
){
    if( strategy_with_goal ){
        strategy_with_goal->assign_behavior_to_robots(
            assign_behavior, time, dt
        );
    }
    for( std::shared_ptr<Strategy> & elem : strategies_without_goal ){
        elem->assign_behavior_to_robots(
            assign_behavior, time, dt
        );
    }
}



}
}
