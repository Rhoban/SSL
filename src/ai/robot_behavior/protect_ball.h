#ifndef __ROBOT_BEHAVIOR__PROTECTBALL__H__
#define __ROBOT_BEHAVIOR__PROTECTBALL__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class ProtectBall : public RobotBehavior  {
    private:
      double radius;

      ConsignFollower* follower;

    public:
        ProtectBall(Ai::AiData& ai_data);

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );

        void declare_radius( double radius = 0.5 );

	virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~ProtectBall();
};

};
}; //Namespace Rhoban

#endif
