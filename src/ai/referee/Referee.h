#ifndef __REFEREE__H__
#define __REFEREE__H__

#include <RefereeClient.h>
#include <core/machine_state.h>
#include <math/circular_vector.h>

namespace RhobanSSL {

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
    RefereeClient referee;
    Referee_data referee_data;

    typedef construct_machine_state_infrastructure<
        std::string, Referee_data, Referee_data
    > machine_infrastructure;

    machine_infrastructure::MachineState machine_state;

    void extract_data();
    void save_last_time_stamps();

public:
    Referee();

    void update( double time );
};

}

#endif
