#ifndef __ROBOT_BEHAVIOR__STRIKER__H__
#define __ROBOT_BEHAVIOR__STRIKER__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Striker : public RobotBehavior  {
    private:
	ConsignFollower* follower;

    public:
        Striker(Ai::AiData& ai_data);

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );

	virtual Control control() const;

	virtual ~Striker();
};

};
}; //Namespace Rhoban

#endif
