#ifndef __REFEREE__H__
#define __REFEREE__H__

#include <RefereeClient.h>

namespace RhobanSSL {

class Referee {
private:
    RefereeClient referee;
    double last_time;

public:
    void update( double time );

};

}

#endif
