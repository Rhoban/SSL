#ifndef __MANAGER__EXAMPLE__H__
#define __MANAGER__EXAMPLE__H__

#include <manager/Manager.h>
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Example : public Manager {
    private:
    const Referee & referee;

    unsigned int last_referee_changement;
            
    std::list<std::string> future_strats;
    
    public:

    Example(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Example();

};

};
};

#endif
