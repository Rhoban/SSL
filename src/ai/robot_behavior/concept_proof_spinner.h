#ifndef __ROBOT_BEHAVIOR__CONCEPT_PROOF_SPINNER__H__
#define __ROBOT_BEHAVIOR__CONCEPT_PROOF_SPINNER__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Concept_proof_spinner : public RobotBehavior  {
    private:
	ConsignFollower* follower;

    bool go_to_home;
    bool save_ball_position;
    rhoban_geometry::Point ball_pos;

    public:
    Concept_proof_spinner(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

	virtual Control control() const;

	virtual ~Concept_proof_spinner();
};

};
}; //Namespace Rhoban

#endif
