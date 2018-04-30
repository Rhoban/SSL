#ifndef __MANAGER__MANUAL__H__
#define __MANAGER__MANUAL__H__

#include "Manager.h"

namespace RhobanSSL {
namespace Manager {

class Manual : public Manager {
    private:
    
    bool strategy_was_assigned;
    
    public:
    Manual( Ai::AiData & game_state );
   
    void update(double time);

    virtual ~Manual();
};

};
};

#endif
