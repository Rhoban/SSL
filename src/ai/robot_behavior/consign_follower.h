#ifndef __ROBOT_BEHAVIOR__CONSIGN_FOLLOWER__H__
#define __ROBOT_BEHAVIOR__CONSIGN_FOLLOWER__H__

#include "robot_behavior.h"

namespace RhobanSSL {
namespace Robot_behavior {

class ConsignFollower : public RobotBehavior {
    public:
    ConsignFollower( Ai::AiData & ai_data );
    
    virtual void set_following_position(
        const Vector2d & position_to_follow,
        const ContinuousAngle & angle
    ) = 0;

    void avoid_the_ball( bool value );

    virtual ~ConsignFollower();
};

};
}; //Namespace Rhoban

#endif
