#ifndef __MANAGER__MATCH__H__
#define __MANAGER__MATCH__H__

#include "Manager.h"
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Match : public Manager {
    private:
    const Referee & referee;

    unsigned int last_referee_changement;
    
    public:

    Match(
        Ai::AiData & game_state,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Match();
};

};
};

#endif
