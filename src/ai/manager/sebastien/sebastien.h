#ifndef __MANAGER__SEBASTIEN__H__
#define __MANAGER__SEBASTIEN__H__

#include <manager/Manager.h>
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Sebastien : public Manager {
    private:
    const Referee & referee;

    unsigned int last_referee_changement;
            
    std::list<std::string> future_strats;
    
    public:

    Sebastien(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Sebastien();

};

};
};

#endif
