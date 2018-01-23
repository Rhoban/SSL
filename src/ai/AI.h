#pragma once

#include "AICommander.h"
#include "AIVisionClient.h"
#include "robot_behavior.h"

namespace RhobanSSL
{

    struct Measure {
        struct Movment_sample {
            double time;
            Eigen::Vector2d position;
            double orientation;

            Movment_sample() : 
                time(0.0), position(0.0, 0.0), orientation(0.0)
            { };
        };

        Movment_sample movment[3];

        void update(
            double time, const Eigen::Vector2d& position, double orientation
        ){
            movment[2] = movment[1];
            movment[1] = movment[0];
            movment[0].time = time;
            movment[0].position = position;
            movment[0].orientation = orientation;
        };

        bool is_valid() const {
            return ( time(2) < time(1) ) and ( time(1) < time(0) );
        };

        double time(unsigned int i=0) const {
            assert( i < 3 );
            return movment[i].time;
        };

        const Eigen::Vector2d & position(unsigned int i=0) const {
            assert( i < 3 );
            return movment[i].position;
        };

        double orientation(unsigned int i=0) const {
            assert( i < 3 );
            return movment[i].orientation;
        };

        double dt( unsigned int i=0 ) const {
            assert( i < 2);
            return movment[i].time - movment[i+1].time;
        }

        Eigen::Vector2d velocity_translation( unsigned int i=0) const {
            return ( position(i) - position(i+1) )/ dt(i);
        };

        Eigen::Vector2d acceleration_translation() const {
            unsigned int i=0;
            return ( velocity_translation(i) - velocity_translation(i+1) )/ dt(i);
        };


        double velocity_rotation( unsigned int i=0) const {
            return ( orientation(i) - orientation(i+1) )/ dt(i);
        };

        double acceleration_rotation() const {
            unsigned int i=0;
            return ( velocity_rotation(i) - velocity_rotation(i+1) )/ dt(i);
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

        bool enable_kicking;

        AICommander *commander;
        AIVisionClient *vision;
        Goalie goalie;
        Shooter shooter;
        Measure shooter_measure;
        
        double max_velocity_t;
        double max_velocity_r;
        double max_acceleration_t;
        double max_acceleration_r;

        Control update_robot( 
            RobotBehavior & robot_behavior,
            double time, GameState::Robot & robot, GameState::Ball & ball
        );

        void prepare_to_send_control( int robot_id, Control control );
    };
};
