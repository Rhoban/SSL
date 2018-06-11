#ifndef __ROBOT_BEHAVIOR__PASS__H__
#define __ROBOT_BEHAVIOR__PASS__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Pass : public RobotBehavior  {
    private:
      int robot_to_pass_id;
      Vision::Team robot_to_pass_team;

      ConsignFollower* follower;

    public:
        Pass(Ai::AiData& ai_data);

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );
        //TODO: portée des variables ?
        void declare_robot_to_pass( int robot_id, Vision::Team team = Vision::Team::Ally );


	virtual Control control() const;

    virtual RhobanSSLAnnotation::Annotations get_annotations() const;

	virtual ~Pass();
};

};
}; //Namespace Rhoban

#endif
