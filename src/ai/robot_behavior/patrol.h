/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __ROBOT_BEHAVIOR__PATROL__H__
#define __ROBOT_BEHAVIOR__PATROL__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Patrol : public RobotBehavior  {
    private:
    ConsignFollower* follower;
    int zone;
    bool _see_the_ball;
    std::vector< std::pair<rhoban_geometry::Point, ContinuousAngle> > traject;
    double waiting_time;
    double last_time;
    bool it_s_time_to_change_the_zone;
    bool reverse_circuit;

    public:
    Patrol(Ai::AiData& ai_data);

    void see_the_ball(bool value);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

    void set_reverse( bool reverse_circuit );

    static Patrol* two_way_trip( Ai::AiData& ai_data );
    static Patrol* two_way_trip_on_width( Ai::AiData& ai_data, bool ally_side );
    static Patrol* two_way_trip_on_border( Ai::AiData& ai_data, bool left);
    static Patrol* tour_of_the_field( Ai::AiData& ai_data, bool reverse_circuit = false );
    static Patrol* triangle( Ai::AiData& ai_data );
    static Patrol* test_translation_for_pid( Ai::AiData& ai_data );	
    static Patrol* test_rotation_for_pid( Ai::AiData& ai_data );	

    static Patrol* test_NW_rotation_for_pid( Ai::AiData& ai_data );	
    static Patrol* test_NE_rotation_for_pid( Ai::AiData& ai_data );	
    static Patrol* test_SW_rotation_for_pid( Ai::AiData& ai_data );	
    static Patrol* test_SE_rotation_for_pid( Ai::AiData& ai_data );	

    static Patrol* test_NW_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_NE_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_SW_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_SE_translation_for_pid( Ai::AiData& ai_data );

    static Patrol* test_N_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_E_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_W_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_S_translation_for_pid( Ai::AiData& ai_data );

    static Patrol* test_SW_NW_translation_for_pid( Ai::AiData& ai_data );
    static Patrol* test_NW_SE_translation_for_pid( Ai::AiData& ai_data );


    void set_traject( const std::vector< std::pair<rhoban_geometry::Point, ContinuousAngle> > & traject );
    void set_traject( const std::vector< rhoban_geometry::Point > & traject );

    virtual Control control() const;
    void set_waiting_time( double time );
    
    RhobanSSLAnnotation::Annotations get_annotations() const;

    virtual ~Patrol();

};

};
}; //Namespace Rhoban

#endif
