#ifndef __GAME_INFORMATIONS__H__
#define __GAME_INFORMATIONS__H__

#include <AiData.h>

namespace RhobanSSL {

class GameInformations {
    private:
    Ai::AiData & ai_data;

    public:
    GameInformations( Ai::AiData & ai_data );
    virtual ~GameInformations();

    double time() const;
    rhoban_geometry::Point ally_goal_center() const ;
    rhoban_geometry::Point oponent_goal_center() const ;
    rhoban_geometry::Point center_mark() const ;
    rhoban_geometry::Point oponent_corner_right() const ;
    rhoban_geometry::Point oponent_corner_left() const ;
    const Ai::Robot & get_robot( int robot_id, Vision::Team team = Vision::Team::Ally ) const ;
};


}

#endif
