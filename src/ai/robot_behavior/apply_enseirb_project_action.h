#ifndef __ROBOT_BEHAVIOR__APPLY_ENSEIRB_PROJECT_ACTION__H__
#define __ROBOT_BEHAVIOR__APPLY_ENSEIRB_PROJECT_ACTION__H__

#include "position_follower.h"
#include <strategy/enseirb_projects/api.h>

namespace RhobanSSL
{
namespace Robot_behavior {

class Apply_enseirb_project_action : public PositionFollower {
    private:
        const Action & action; 
    public:
        Apply_enseirb_project_action( const Action& action, double time, double dt );

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );
        
        virtual Control control() const;
};

};
}; //Namespace Rhoban

#endif
