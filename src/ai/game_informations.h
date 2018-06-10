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
    const Ai::Robot & get_robot( int robot_id, Vision::Team team = Vision::Team::Ally ) const ;

    std::vector<rhoban_geometry::Point> center_quarter_field() const ;

    rhoban_geometry::Point center_ally_field() const ;
    rhoban_geometry::Point center_opponent_field() const ;
    double get_robot_radius() const;
    double get_ball_radius() const;

};


}

#endif
