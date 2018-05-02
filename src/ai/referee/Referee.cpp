#include "Referee.h"
#include <debug.h>
#include <core/print_collection.h>

namespace RhobanSSL {

const std::string Referee_Id::STATE_INIT = "init"; 
const std::string Referee_Id::STATE_HALTED = "halted";
const std::string Referee_Id::STATE_STOPPED = "stopped"; 
const std::string Referee_Id::STATE_PREPARE_KICKOFF = "prepare_kickoff";
const std::string Referee_Id::STATE_PREPARE_PENALTY = "prepare_penalty";
const std::string Referee_Id::STATE_RUNNING = "running";
const std::string Referee_Id::STATE_TIMEOUT = "timeout";

const std::string Referee_Id::EDGE_INIT_TO_STOPPED = "stop_game_u";
const std::string Referee_Id::EDGE_HALTED_TO_STOPPED = "stop_game_h"; 
const std::string Referee_Id::EDGE_PREPARE_KICKOFF_TO_STOPPED = "stop_game_pk"; 
const std::string Referee_Id::EDGE_PREPARE_PENALTY_TO_STOPPED = "stop_game_pp"; 
const std::string Referee_Id::EDGE_RUNNING_TO_STOPPED = "stop_game_r"; 
const std::string Referee_Id::EDGE_TIMEOUT_TO_STOPPED = "stop_game_t"; 

const std::string Referee_Id::EDGE_INIT_TO_HALTED = "halt_game_u"; 
const std::string Referee_Id::EDGE_STOPPED_TO_HALTED = "halt_game_s"; 
const std::string Referee_Id::EDGE_PREPARE_KICKOFF_TO_HALTED = "halt_game_pk"; 
const std::string Referee_Id::EDGE_PREPARE_PENALTY_TO_HALTED = "halt_game_pp"; 
const std::string Referee_Id::EDGE_RUNNING_TO_HALTED = "halt_game_r"; 
const std::string Referee_Id::EDGE_TIMEOUT_TO_HALTED = "halt_game_t"; 

const std::string Referee_Id::EDGE_TIMEOUT_START = "timeout_start"; 
const std::string Referee_Id::EDGE_FORCE_START = "force_start"; 
const std::string Referee_Id::EDGE_KICKOFF_YELLOW = "kickoff_yellow"; 
const std::string Referee_Id::EDGE_KICKOFF_BLUE = "kickoff_blue"; 
const std::string Referee_Id::EDGE_PENALTY = "penalty"; 
const std::string Referee_Id::EDGE_INDIRECT = "indirect";
const std::string Referee_Id::EDGE_DIRECT = "direct";

const std::string Referee_Id::EDGE_NORMAL_START_FOR_KICKOFF = "normal_start_k";
const std::string Referee_Id::EDGE_NORMAL_START_FOR_PENALTY = "normal_start_p";

const std::string Referee_Id::EDGE_TIMEOUT_RESUME = "timeout_resume";
const std::string Referee_Id::EDGE_GOAL = "goal";





Referee_data::Referee_data():
    datas(2), last_time(0.0), last_command_counter(0)
{ }

const SSL_Referee& Referee_data::current() const {
    return datas[0];
}

const SSL_Referee& Referee_data::old() const {
    return datas[1];
}

bool Referee_data::command_is_new() const {
    return (
        last_time < current().packet_timestamp()
    ) and (
        last_command_counter < current().command_counter()
    );
}



template <SSL_Referee_Command E>
bool command_is_(
    const Referee_data & referee_data, 
    unsigned int run_number, unsigned int atomic_run_number
){
    return (
        referee_data.command_is_new() 
    ) and (
        referee_data.current().command() == E
    ); 
}

template <SSL_Referee_Command E1, SSL_Referee_Command E2>
bool command_is_one_of_(
    const Referee_data & referee_data, 
    unsigned int run_number, unsigned int atomic_run_number
){
    return (
        referee_data.command_is_new() 
    ) and (
        ( referee_data.current().command() == E1 )
        or 
        ( referee_data.current().command() == E2 ) 
    ); 
}



Referee::Referee():
    edge_entropy_number(0),
    machine_state(referee_data, referee_data)
{
    machine_state
        .add_state( Referee_Id::STATE_INIT ) // Referee is lost
        .add_state( Referee_Id::STATE_HALTED )
        .add_state( Referee_Id::STATE_STOPPED )
        .add_state( Referee_Id::STATE_PREPARE_KICKOFF )
        .add_state( Referee_Id::STATE_PREPARE_PENALTY )
        .add_state( Referee_Id::STATE_RUNNING )
        .add_state( Referee_Id::STATE_TIMEOUT )
        .add_init_state( Referee_Id::STATE_INIT )
    ;


    machine_state  
        .add_edge(
            Referee_Id::EDGE_INIT_TO_STOPPED,
            Referee_Id::STATE_INIT, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            Referee_Id::EDGE_HALTED_TO_STOPPED,
            Referee_Id::STATE_HALTED, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            Referee_Id::EDGE_PREPARE_KICKOFF_TO_STOPPED,
            Referee_Id::STATE_PREPARE_KICKOFF, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            Referee_Id::EDGE_PREPARE_PENALTY_TO_STOPPED,
            Referee_Id::STATE_PREPARE_PENALTY, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            Referee_Id::EDGE_RUNNING_TO_STOPPED,
            Referee_Id::STATE_RUNNING, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            Referee_Id::EDGE_TIMEOUT_TO_STOPPED,
            Referee_Id::STATE_TIMEOUT, Referee_Id::STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
    ;
   
    machine_state  
        .add_edge(
            Referee_Id::EDGE_INIT_TO_HALTED,
            Referee_Id::STATE_INIT, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            Referee_Id::EDGE_STOPPED_TO_HALTED,
            Referee_Id::STATE_STOPPED, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            Referee_Id::EDGE_PREPARE_KICKOFF_TO_HALTED,
            Referee_Id::STATE_PREPARE_KICKOFF, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            Referee_Id::EDGE_PREPARE_PENALTY_TO_HALTED,
            Referee_Id::STATE_PREPARE_PENALTY, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            Referee_Id::EDGE_RUNNING_TO_HALTED,
            Referee_Id::STATE_RUNNING, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            Referee_Id::EDGE_TIMEOUT_TO_HALTED,
            Referee_Id::STATE_TIMEOUT, Referee_Id::STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
    ;

    machine_state.add_edge(
        Referee_Id::EDGE_TIMEOUT_START,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_TIMEOUT,
        command_is_one_of_<
            SSL_Referee::TIMEOUT_BLUE, SSL_Referee::TIMEOUT_YELLOW
        >
    );
    machine_state.add_edge(
        Referee_Id::EDGE_FORCE_START,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_RUNNING,
        command_is_<SSL_Referee::FORCE_START>
    );
    machine_state.add_edge(
        Referee_Id::EDGE_KICKOFF_YELLOW,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_PREPARE_KICKOFF,
        command_is_< SSL_Referee::PREPARE_KICKOFF_YELLOW>,
        [&](
            const Referee_data & referee_data, 
            unsigned int run_number, unsigned int atomic_run_number
        ){
            team_having_kickoff = Ai::Yellow;
        }
    );
    machine_state.add_edge(
        Referee_Id::EDGE_KICKOFF_BLUE,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_PREPARE_KICKOFF,
        command_is_<SSL_Referee::PREPARE_KICKOFF_BLUE>,
        [&](
            const Referee_data & referee_data, 
            unsigned int run_number, unsigned int atomic_run_number
        ){
            team_having_kickoff = Ai::Blue;
        }
    );
    machine_state.add_edge(
        Referee_Id::EDGE_PENALTY,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_PREPARE_PENALTY,
        command_is_one_of_<
            SSL_Referee::PREPARE_PENALTY_BLUE, SSL_Referee::PREPARE_PENALTY_YELLOW
        >
    );
    machine_state.add_edge(
        Referee_Id::EDGE_INDIRECT,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_RUNNING,
        command_is_one_of_<
            SSL_Referee::INDIRECT_FREE_BLUE, SSL_Referee::INDIRECT_FREE_YELLOW
        >
    );
    machine_state.add_edge(
        Referee_Id::EDGE_DIRECT,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_RUNNING,
        command_is_one_of_<
            SSL_Referee::DIRECT_FREE_BLUE, SSL_Referee::DIRECT_FREE_YELLOW
        >
    );

    machine_state.add_edge(
        Referee_Id::EDGE_NORMAL_START_FOR_KICKOFF,
        Referee_Id::STATE_PREPARE_KICKOFF, Referee_Id::STATE_RUNNING,
        command_is_<SSL_Referee::NORMAL_START>
    );
    machine_state.add_edge(
        Referee_Id::EDGE_NORMAL_START_FOR_PENALTY,
        Referee_Id::STATE_PREPARE_PENALTY, Referee_Id::STATE_RUNNING,
        command_is_<SSL_Referee::NORMAL_START>
    );

    machine_state.add_edge(
        Referee_Id::EDGE_TIMEOUT_RESUME,
        Referee_Id::STATE_HALTED, Referee_Id::STATE_TIMEOUT,
        command_is_one_of_<
            SSL_Referee::TIMEOUT_BLUE, SSL_Referee::TIMEOUT_YELLOW
        >
    );

    machine_state.add_edge(
        Referee_Id::EDGE_GOAL,
        Referee_Id::STATE_STOPPED, Referee_Id::STATE_STOPPED,
        command_is_one_of_<
            SSL_Referee::GOAL_BLUE, SSL_Referee::GOAL_YELLOW
        >
    );

    machine_state.execute_at_each_edge(
        [&](
             std::string edge_id, Referee_data & state_data, Referee_data & edge_data,
             unsigned int run_number, unsigned int atomic_run_number
        ){
             edge_entropy_number += 1;
        }
    );

    machine_state.start();
}

void Referee::extract_data(){
    SSL_Referee data = referee.getData(); 
        // Use this function just one time if you want to avoir thread 
        // issue.
    if( referee_data.last_time < data.packet_timestamp() ){
        referee_data.datas.insert( data );
        if( data.has_blueteamonpositivehalf() ){
            blueTeamOnPositiveHalf = data.blueteamonpositivehalf();
        }
    }
}

void Referee::save_last_time_stamps(){
    if( referee_data.last_time < referee_data.current().packet_timestamp() ){
        referee_data.last_time = referee_data.current().packet_timestamp();
        
        if( 
            referee_data.last_command_counter < 
            referee_data.current().command_counter() 
        ){ 
            referee_data.last_command_counter = referee_data.current().command_counter();
            DEBUG( "Command is : " << referee_data.current().command() );
            DEBUG( "State is : " << machine_state.current_states() );
        }
    }
}


void Referee::update( double time ){
    extract_data();
    machine_state.run();
    assert( machine_state.current_states().size() == 1 );
    save_last_time_stamps();
}


unsigned int Referee::edge_entropy() const {
    return edge_entropy_number;
}
const Referee::ID & Referee::get_state() const {
    assert( machine_state.current_states().size() == 1 );
    return *( machine_state.current_states().begin());
}

Ai::Team Referee::kickoff_team() const {
    return team_having_kickoff;
}

bool Referee::blue_have_it_s_goal_on_positive_x_axis() const {
    return blueTeamOnPositiveHalf;
}

int Referee::yellow_goalie_id() const {
    return referee_data.current().yellow().goalie();
}

int Referee::blue_goalie_id() const {
    return referee_data.current().blue().goalie();
}

Ai::Team Referee::get_team_color( const std::string & team_name ) const {
    if( referee_data.current().yellow().name() == team_name ){
        return Ai::Yellow;
    }
    if( referee_data.current().blue().name() == team_name ){
        return Ai::Blue;
    }
    return Ai::Unknown;
};    

}
