#ifndef __ROBOT_BEHAVIOR__DO_NOTHING__H__
#define __ROBOT_BEHAVIOR__DO_NOTHING__H__

#include "robot_behavior.h"

namespace RhobanSSL {
namespace Robot_behavior {

class DoNothing : public RobotBehavior {
    public:
        DoNothing( Ai::AiData& ai_data ); 

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;
};

};
}; //Namespace Rhoban

#endif
