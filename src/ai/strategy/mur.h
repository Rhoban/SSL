#ifndef __STRATEGY__MUR__H__
#define __STRATEGY__MUR__H__

#include "Strategy.h"

namespace RhobanSSL {
namespace Strategy {

class Mur : public Strategy {
    private:
    bool behaviors_are_assigned;

    public:
    Mur(Ai::AiData & ai_data);
    virtual ~Mur();        

    virtual int min_robots() const;
    virtual int max_robots() const;
    virtual Goalie_need needs_goalie() const;

    static const std::string name;

    virtual void start(double time);
    virtual void stop(double time);

    virtual void update(double time);

    virtual void assign_behavior_to_robots(
        std::function<
            void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
        > assign_behavior,
        double time, double dt
    );

    virtual std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    > get_starting_positions( int number_of_avalaible_robots ) ;  
    virtual bool get_starting_position_for_goalie(
        rhoban_geometry::Point & linear_position, 
        ContinuousAngle & angular_position
    ) ;  


};

};
};
#endif
