#ifndef __ROBOT_BEHAVIOR__ROBOT_FOLLOWER__H__
#define __ROBOT_BEHAVIOR__ROBOT_FOLLOWER__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class RobotFollower : public RobotBehavior  {
    private:
    int robot_to_follow_id;
    Vision::Team robot_to_follow_team;

    Vector2d translation;
    Vision::Team team;
	
    ConsignFollower* follower;

    public:
    RobotFollower(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

    void declare_robot_to_follow( int robot_id, const Vector2d & translation, Vision::Team team = Vision::Team::Ally );

	virtual Control control() const;

	virtual ~RobotFollower();
};

};
}; //Namespace Rhoban

#endif
