#ifndef __ROBOT_BEHAVIOR__EXAMPLER__H__
#define __ROBOT_BEHAVIOR__EXAMPLER__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Example : public RobotBehavior  {
    private:
	ConsignFollower* follower;

    public:
    Example(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

	virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Example();
};

};
}; //Namespace Rhoban

#endif
