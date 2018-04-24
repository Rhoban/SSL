#include "Referee.h"
#include <debug.h>
#include <core/print_collection.h>

#define STATE_INIT "init" 
#define STATE_HALTED "halted" 
#define STATE_STOPPED "stopped" 
#define STATE_PREPARE_KICKOFF "prepare_kickoff" 
#define STATE_PREPARE_PENALTY "prepare_penalty" 
#define STATE_RUNNING "running" 
#define STATE_TIMEOUT "timeout" 

#define EDGE_INIT_TO_STOPPED "stop_game_u" 
#define EDGE_HALTED_TO_STOPPED "stop_game_h" 
#define EDGE_PREPARE_KICKOFF_TO_STOPPED "stop_game_pk" 
#define EDGE_PREPARE_PENALTY_TO_STOPPED "stop_game_pp" 
#define EDGE_RUNNING_TO_STOPPED "stop_game_r" 
#define EDGE_TIMEOUT_TO_STOPPED "stop_game_t" 

#define EDGE_INIT_TO_HALTED "halt_game_u" 
#define EDGE_STOPPED_TO_HALTED "halt_game_s" 
#define EDGE_PREPARE_KICKOFF_TO_HALTED "halt_game_pk" 
#define EDGE_PREPARE_PENALTY_TO_HALTED "halt_game_pp" 
#define EDGE_RUNNING_TO_HALTED "halt_game_r" 
#define EDGE_TIMEOUT_TO_HALTED "halt_game_t" 

#define EDGE_TIMEOUT_START "timeout_start" 
#define EDGE_FORCE_START "force_start" 
#define EDGE_KICKOFF "kickoff" 
#define EDGE_PENALTY "penalty" 
#define EDGE_INDIRECT "indirect"
#define EDGE_DIRECT "direct"

#define EDGE_NORMAL_START_FOR_KICKOFF "normal_start_k"
#define EDGE_NORMAL_START_FOR_PENALTY "normal_start_p"

#define EDGE_TIMEOUT_RESUME "timeout_resume"
#define EDGE_GOAL "goal"

namespace RhobanSSL {

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
    machine_state(referee_data, referee_data)
{
    machine_state
        .add_state( STATE_INIT ) // Referee is lost
        .add_state( STATE_HALTED )
        .add_state( STATE_STOPPED )
        .add_state( STATE_PREPARE_KICKOFF )
        .add_state( STATE_PREPARE_PENALTY )
        .add_state( STATE_RUNNING )
        .add_state( STATE_TIMEOUT )
        .add_init_state( STATE_INIT )
    ;


    machine_state  
        .add_edge(
            EDGE_INIT_TO_STOPPED,
            STATE_INIT, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            EDGE_HALTED_TO_STOPPED,
            STATE_HALTED, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            EDGE_PREPARE_KICKOFF_TO_STOPPED,
            STATE_PREPARE_KICKOFF, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            EDGE_PREPARE_PENALTY_TO_STOPPED,
            STATE_PREPARE_PENALTY, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            EDGE_RUNNING_TO_STOPPED,
            STATE_RUNNING, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
        .add_edge(
            EDGE_TIMEOUT_TO_STOPPED,
            STATE_TIMEOUT, STATE_STOPPED,
            command_is_<SSL_Referee::STOP>
        )
    ;
   
    machine_state  
        .add_edge(
            EDGE_INIT_TO_HALTED,
            STATE_INIT, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            EDGE_STOPPED_TO_HALTED,
            STATE_STOPPED, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            EDGE_PREPARE_KICKOFF_TO_HALTED,
            STATE_PREPARE_KICKOFF, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            EDGE_PREPARE_PENALTY_TO_HALTED,
            STATE_PREPARE_PENALTY, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            EDGE_RUNNING_TO_HALTED,
            STATE_RUNNING, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
        .add_edge(
            EDGE_TIMEOUT_TO_HALTED,
            STATE_TIMEOUT, STATE_HALTED,
            command_is_<SSL_Referee::HALT>
        )
    ;

    machine_state.add_edge(
        EDGE_TIMEOUT_START,
        STATE_STOPPED, STATE_TIMEOUT,
        command_is_one_of_<
            SSL_Referee::TIMEOUT_BLUE, SSL_Referee::TIMEOUT_YELLOW
        >
    );
    machine_state.add_edge(
        EDGE_FORCE_START,
        STATE_STOPPED, STATE_RUNNING,
        command_is_<SSL_Referee::FORCE_START>
    );
    machine_state.add_edge(
        EDGE_KICKOFF,
        STATE_STOPPED, STATE_PREPARE_KICKOFF,
        command_is_one_of_<
            SSL_Referee::PREPARE_KICKOFF_BLUE, SSL_Referee::PREPARE_KICKOFF_YELLOW
        >
    );
    machine_state.add_edge(
        EDGE_PENALTY,
        STATE_STOPPED, STATE_PREPARE_PENALTY,
        command_is_one_of_<
            SSL_Referee::PREPARE_PENALTY_BLUE, SSL_Referee::PREPARE_PENALTY_YELLOW
        >
    );
    machine_state.add_edge(
        EDGE_INDIRECT,
        STATE_STOPPED, STATE_RUNNING,
        command_is_one_of_<
            SSL_Referee::INDIRECT_FREE_BLUE, SSL_Referee::INDIRECT_FREE_YELLOW
        >
    );
    machine_state.add_edge(
        EDGE_DIRECT,
        STATE_STOPPED, STATE_RUNNING,
        command_is_one_of_<
            SSL_Referee::DIRECT_FREE_BLUE, SSL_Referee::DIRECT_FREE_YELLOW
        >
    );

    machine_state.add_edge(
        EDGE_NORMAL_START_FOR_KICKOFF,
        STATE_PREPARE_KICKOFF, STATE_RUNNING,
        command_is_<SSL_Referee::NORMAL_START>
    );
    machine_state.add_edge(
        EDGE_NORMAL_START_FOR_PENALTY,
        STATE_PREPARE_PENALTY, STATE_RUNNING,
        command_is_<SSL_Referee::NORMAL_START>
    );

    machine_state.add_edge(
        EDGE_TIMEOUT_RESUME,
        STATE_HALTED, STATE_TIMEOUT,
        command_is_one_of_<
            SSL_Referee::TIMEOUT_BLUE, SSL_Referee::TIMEOUT_YELLOW
        >
    );

    machine_state.add_edge(
        EDGE_GOAL,
        STATE_STOPPED, STATE_STOPPED,
        command_is_one_of_<
            SSL_Referee::GOAL_BLUE, SSL_Referee::GOAL_YELLOW
        >
    );


    machine_state.start();
}

void Referee::extract_data(){
    SSL_Referee data = referee.getData(); 
        // Use this function just one time if you want to avoir thread 
        // issue.
    if( referee_data.last_time < data.packet_timestamp() ){
        referee_data.datas.insert( data );
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

}
