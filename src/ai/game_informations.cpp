#include "game_informations.h"

namespace RhobanSSL {

GameInformations::GameInformations( Ai::AiData & ai_data ):
    ai_data( ai_data)
{ }

GameInformations::~GameInformations(){ }

double GameInformations::time() const {
    return ai_data.time;
}


rhoban_geometry::Point GameInformations::ally_goal_center() const {
    return  rhoban_geometry::Point( - ai_data.field.fieldLength/2.0, 0.0 );

}

rhoban_geometry::Point GameInformations::oponent_goal_center() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, 0.0 );
}

rhoban_geometry::Point GameInformations::center_mark() const {
    return rhoban_geometry::Point( 0.0, 0.0 );
}

rhoban_geometry::Point GameInformations::oponent_corner_right() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, - ai_data.field.fieldWidth/2.0 );
}

rhoban_geometry::Point GameInformations::oponent_corner_left() const {
    return rhoban_geometry::Point( ai_data.field.fieldLength/2.0, ai_data.field.fieldWidth/2.0 );
}

const Ai::Robot & GameInformations::get_robot( int robot_id, Vision::Team team ) const {
    return ai_data.robots.at(team).at(robot_id);
}


};
