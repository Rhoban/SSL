#ifndef __MANAGER__MANUAL__H__
#define __MANAGER__MANUAL__H__

#include "Manager.h"

namespace RhobanSSL {
namespace Manager {

class Manual : public Manager {
    private:
    
    bool strategy_was_assigned;
    
    Ai::Team team_color;
    bool goal_to_positive_axis;
    int ally_goalie_id;
    int oponnent_goalie_id;

    void assign_point_of_view_and_goalie();

    
    public:
    Manual( Ai::AiData & ai_data );

    void set_team_color( Ai::Team team_color );
    void define_goal_to_positive_axis(bool value = true);
    void set_ally_goalie_id( int id );
    void set_oponnent_goalie_id( int id );

    void update(double time);

    virtual ~Manual();
};

};
};

#endif
