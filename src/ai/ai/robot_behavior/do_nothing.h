#ifndef __DO_NOTHING__H__
#define __DO_NOTHING__H__

#include "robot_behavior.h"

namespace RhobanSSL {

class DoNothing : public RobotBehavior {
    public:
        DoNothing(); 

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;
};

}; //Namespace Rhoban

#endif
