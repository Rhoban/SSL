#ifndef __MANAGER__MATCH__H__
#define __MANAGER__MATCH__H__

#include "Manager.h"
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Match : public Manager {
    private:
    const Referee & referee;
    std::vector<int> enseirb_robots;

    unsigned int last_referee_changement;
    
    public:

    Match(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Match();

    private:
    std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    > starting_positions;
    bool goal_has_to_be_placed;
    rhoban_geometry::Point goalie_linear_position;
    ContinuousAngle goalie_angular_position;

    void aggregate_all_starting_position_of_all_strategies();
    void declare_robot_positions_in_the_placer();
    void place_all_the_robots(double time);

};

};
};

#endif
