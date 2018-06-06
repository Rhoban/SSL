#ifndef __MANAGER__EXAMPLE_FOR_TESTING_ROBOT_BEHAVIOR__H__
#define __MANAGER__EXAMPLE_FOR_TESTING_ROBOT_BEHAVIOR__H__

#include "Manager.h"
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class Example_for_testing_robot_behaviors : public Manager {
    private:
    const Referee & referee;

    unsigned int last_referee_changement;
            
    std::list<std::string> future_strats;
    
    public:

    Example_for_testing_robot_behaviors(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~Example_for_testing_robot_behaviors();

};

};
};

#endif
