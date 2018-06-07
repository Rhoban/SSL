#ifndef __ROBOT_BEHAVIOR__PASSIVE_DEFENSOR__H__
#define __ROBOT_BEHAVIOR__PASSIVE_DEFENSOR__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Passive_defensor : public RobotBehavior  {
    private:
	ConsignFollower* follower;
    int robot_to_obstale_id;
    Vision::Team robot_to_obstale_team;
    double barycenter;

    public:
    Passive_defensor(Ai::AiData& ai_data);

    virtual void update(
        double time,
        const Ai::Robot & robot,
        const Ai::Ball & ball
    );

    void set_robot_to_obstacle( int robot_id, Vision::Team team = Vision::Team::Opponent );
    void set_barycenter( double barycenter );
    //void obstacle_the_robot_closed_to_the_ally_goal_line();

	virtual Control control() const;

	virtual ~Passive_defensor();
};

};
}; //Namespace Rhoban

#endif
