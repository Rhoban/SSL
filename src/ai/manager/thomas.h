#ifndef __MANAGER__THOMAS__H__
#define __MANAGER__THOMAS__H__

#include "Manager.h"
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Thomas : public Manager {
    private:
    const Referee & referee;

    unsigned int last_referee_changement;
            
    std::list<std::string> future_strats;
    
    public:

    Thomas(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Thomas();

};

};
};

#endif
