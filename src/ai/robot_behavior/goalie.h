#ifndef __GOALIE__H__
#define __GOALIE__H__

#include "position_follower.h"

namespace RhobanSSL
{

class Goalie : public PositionFollower {
    private:
        PositionFollower follower();

        Vector2d left_post_position; 
        Vector2d right_post_position;
        Vector2d goal_center;
        Vector2d waiting_goal_position;

        double goalie_radius;
        double penalty_rayon;

        static Vector2d calculate_goal_position(
            const Vector2d & ball_position,
            const Vector2d & poteau_droit,
            const Vector2d & poteau_gauche,
            double goalie_radius
        );

    public:
        Goalie(
            const Vector2d & left_post_position,
            const Vector2d & right_post_position,
            const Vector2d & waiting_goal_position,
            double penalty_rayon,
            double goalie_radius,
            double time, double dt
        );

        virtual void update(
            double time,
            const Ai::Robot & robot,
            const Ai::Ball & ball
        );
};

}; //Namespace Rhoban

#endif
