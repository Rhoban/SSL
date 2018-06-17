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

#ifndef __REFEREE__H__
#define __REFEREE__H__

#include <RefereeClient.h>
#include <core/machine_state.h>
#include <math/circular_vector.h>
#include <AiData.h>



namespace RhobanSSL {

struct Referee_Id {
    static const std::string STATE_INIT;
    static const std::string STATE_HALTED;
    static const std::string STATE_STOPPED;
    static const std::string STATE_PREPARE_KICKOFF;
    static const std::string STATE_PREPARE_PENALTY;
    static const std::string STATE_RUNNING;
    static const std::string STATE_TIMEOUT;

    static const std::string EDGE_INIT_TO_STOPPED;
    static const std::string EDGE_HALTED_TO_STOPPED;
    static const std::string EDGE_PREPARE_KICKOFF_TO_STOPPED;
    static const std::string EDGE_PREPARE_PENALTY_TO_STOPPED;
    static const std::string EDGE_RUNNING_TO_STOPPED;
    static const std::string EDGE_TIMEOUT_TO_STOPPED;

    static const std::string EDGE_INIT_TO_HALTED;
    static const std::string EDGE_STOPPED_TO_HALTED;
    static const std::string EDGE_PREPARE_KICKOFF_TO_HALTED;
    static const std::string EDGE_PREPARE_PENALTY_TO_HALTED;
    static const std::string EDGE_RUNNING_TO_HALTED;
    static const std::string EDGE_TIMEOUT_TO_HALTED;

    static const std::string EDGE_TIMEOUT_START;
    static const std::string EDGE_FORCE_START;
    static const std::string EDGE_KICKOFF_YELLOW;
    static const std::string EDGE_KICKOFF_BLUE;
    static const std::string EDGE_PENALTY_YELLOW;
    static const std::string EDGE_PENALTY_BLUE;
    static const std::string EDGE_INDIRECT;
    static const std::string EDGE_DIRECT;

    static const std::string EDGE_NORMAL_START_FOR_KICKOFF;
    static const std::string EDGE_NORMAL_START_FOR_PENALTY;

    static const std::string EDGE_TIMEOUT_RESUME;
    static const std::string EDGE_GOAL;
};


struct Referee_data {

    //datas[0] is the most recent
    //datas[1] the older
    circular_vector<SSL_Referee> datas;

    double last_time;
    uint32_t last_command_counter;

    Referee_data();

    const SSL_Referee& current() const;
    const SSL_Referee& old() const;

    bool command_is_new() const;
};

class Referee {
private:
    bool blueTeamOnPositiveHalf;

    RefereeClient referee;
    Referee_data referee_data;
    unsigned int edge_entropy_number;

    typedef std::string ID;
    typedef construct_machine_state_infrastructure<
        ID, Referee_data, Referee_data
    > machine_infrastructure;

    machine_infrastructure::MachineState machine_state;

    void extract_data();
    void save_last_time_stamps();

    Ai::Team team_having_kickoff;
    Ai::Team team_having_penalty;
public:
    Referee();

    unsigned int edge_entropy() const ;
    const ID & get_state() const ;

    void update( double time );

    Ai::Team kickoff_team() const ;
    Ai::Team penalty_team() const ;

    bool blue_have_it_s_goal_on_positive_x_axis() const;
    Ai::Team get_team_color( const std::string & team_name ) const;

    int yellow_goalie_id() const;
    int blue_goalie_id() const;

    RefereeClient &getRefereeClient();

};

}

#endif
