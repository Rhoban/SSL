#ifndef __ROBOT_BEHAVIOR__POSITION_FOLLOWER__H__
#define __ROBOT_BEHAVIOR__POSITION_FOLLOWER__H__

#include "robot_behavior.h"
#include "consign_follower.h"

namespace RhobanSSL
{
namespace Robot_behavior {


class PositionFollower : public ConsignFollower {
    private:
        Vector2d position;
        ContinuousAngle angle;

        RobotControlWithPositionFollowing robot_control;

    public:
        PositionFollower( Ai::AiData& ai_data, double time, double dt ); 

        void set_translation_pid( double kp, double ki, double kd );
        void set_orientation_pid( double kp, double ki, double kd );

        void set_limits(
            double translation_velocity_limit,
            double rotation_velocity_limit,
            double translation_acceleration_limit,
            double rotation_acceleration_limit
        );

        virtual void set_following_position(
            const Vector2d & position_to_follow,
            const ContinuousAngle & angle
        );

    protected:
        void update_control(double time);

    public:
        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual Control control() const;
};

};
}; //Namespace Rhoban

#endif
