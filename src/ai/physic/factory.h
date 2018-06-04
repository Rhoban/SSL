#ifndef __PHYSIC__FACTORY__H__
#define __PHYSIC__FACTORY__H__

#include "Movement.h"
#include <AiData.h>

namespace RhobanSSL {
namespace physic {

class Factory {
    public:
    static Movement* movement(Ai::AiData & ai_data);
    static Movement* robot_movement(Ai::AiData & ai_data);
    static Movement* ball_movement(Ai::AiData & ai_data);

};

};
};

#endif
