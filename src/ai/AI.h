#pragma once

#include "AICommander.h"
#include "AIVisionClient.h"
#include "movment.h"

// Comment to use COntrol with fixed goal
//#define CURVE_FOLLLOWING

namespace RhobanSSL
{

    struct Translation {
        Eigen::Vector2d position;
        
        Eigen::Vector2d operator()(double u) const {
            return  position + Eigen::Vector2d(u, 0); 
            //return  position + Eigen::Vector2d(u,0.0); 
        };
    };

    struct Rotation {
        double orientation;

        double operator()(double u) const {
            //return  0.0*u + orientation;
            return  3.14159265/2.0*u + orientation;
        };
    };

    class AI
    {
    public:
        AI(AIVisionClient *vision, AICommander *commander);

        void tick();
        void run();
        void stop();

    protected:
        bool running;
        AICommander *commander;
        AIVisionClient *vision;
        #ifdef CURVE_FOLLLOWING
        RobotControlWithCurve control;
        #else
        RobotControlWithPositionFollowing control;
        #endif
        Translation robot_translation;
        Rotation robot_rotation;
    };
}
