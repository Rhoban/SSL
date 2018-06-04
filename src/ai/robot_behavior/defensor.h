#ifndef __ROBOT_BEHAVIOR__DEFENSOR__H__
#define __ROBOT_BEHAVIOR__DEFENSOR__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Defensor : public RobotBehavior  {
    private:
	ConsignFollower* follower;

    public:
        Defensor(Ai::AiData& ai_data);

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );

	virtual Control control() const;

	virtual ~Defensor();
};

};
}; //Namespace Rhoban

#endif
