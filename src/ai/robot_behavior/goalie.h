#ifndef __ROBOT_BEHAVIOR__GOALIE__H__
#define __ROBOT_BEHAVIOR__GOALIE__H__

#include "robot_behavior.h"
#include "factory.h"

namespace RhobanSSL
{
namespace Robot_behavior {

class Goalie : public RobotBehavior  {
    private:
        //PositionFollower follower(); TODO : to remove if not necessary

	ConsignFollower* follower;

        Vector2d left_post_position; 
        Vector2d right_post_position;
        Vector2d goal_center;
        Vector2d waiting_goal_position;

        double goalie_radius;
        double penalty_rayon;

        static Vector2d calculate_goal_position(
            const rhoban_geometry::Point & ball_position,
            const Vector2d & poteau_droit,
            const Vector2d & poteau_gauche,
            double goalie_radius
        );

    public:
        Goalie(Ai::AiData& ai_data);

        Goalie(
            Ai::AiData& ai_data,
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

	virtual Control control() const;

	virtual ~Goalie();
};

};
}; //Namespace Rhoban

#endif
