#include "AI.h"
#include <timing/TimeStamp.hpp>
#include <cmath>
#include <unistd.h>

using namespace Utils::Timing;

namespace RhobanSSL
{

    Eigen::Vector2d calculate_goal_position(
        const Eigen::Vector2d & ball_position,
        const Eigen::Vector2d & poteau_droit,
        const Eigen::Vector2d & poteau_gauche,
        double goal_rayon
    ){
        Eigen::Vector2d R = poteau_droit - ball_position;
        R /= R.norm();
        Eigen::Vector2d L = poteau_gauche - ball_position;
        L /= L.norm();
        Eigen::Matrix2d m;
        m << 
            -R[1], R[0],
             L[1], -L[0];
        return ball_position + m.inverse() * Eigen::Vector2d(goal_rayon, goal_rayon);

    }


    double angle( Eigen::Vector2d direction ){
        double norm = direction.norm();
        if( norm == 0.0 ) return 0.0;
        direction /= norm;
        double angle = std::acos( direction[0] );
        if( direction[1] <= 0 ) return -angle;
        return angle;
    }

    AI::AI(AIVisionClient *vision, AICommander *commander)
    : vision(vision), commander(commander)
    {
        running = true;
    }


    void AI::tick()
    {
        auto gameState = vision->getGameState();
        
        #define SSL_SIMU

        #ifdef SSL_SIMU
            // SSL SIMUL
            Eigen::Vector2d left_goal_position( -4.5, -0.5 );
            Eigen::Vector2d right_goal_position( -4.50, 0.5 );
            Eigen::Vector2d goal_center = ( left_goal_position + right_goal_position)/2.0;
        #else
            // SSL QUALIF
            Eigen::Vector2d left_goal_position( -2.8, -0.31 );
            Eigen::Vector2d right_goal_position( -2.80, 0.29 );
            Eigen::Vector2d goal_center = ( left_goal_position + right_goal_position)/2.0;
        #endif

        #ifdef SSL_SIMU
            auto robot = gameState.robots[GameState::Ally][0];
        #else
            auto robot = gameState.robots[GameState::Ally][5];
        #endif
        
        auto ball = gameState.ball;

         std::cout << "position robot " << robot.position.getX() << ", " << robot.position.getY() << std::endl;
         std::cout << "position ball " << ball.position.getX() << ", " << ball.position.getY() << std::endl;

        if (robot.isOk()) {

            control.set_translation_pid( 0.01, .00, .0);
            control.set_orientation_pid( 0.02, .00, .0);
            if( control.is_static() ){
                Eigen::Vector2d robot_position( 
                    robot.position.getX(), robot.position.getY()
                );
                robot_translation.position = robot_position;
                robot_rotation.orientation = robot.orientation.getSignedValue();

#ifdef CURVE_FOLLLOWING

                // BUG : SI vous obtenez une boucle infini, vous devez bidoullier les paramètres (en baissant l'acceleration).
                //  Le bug sera résolu plus tard.
                double translation_velocity = 0.8;
                double translation_acceleration = 10.0;

                double angular_velocity = 1.5;  
                double angular_acceleration = 20.0;

                double calculus_step = 0.0001;
                double time = TimeStamp::now().getTimeMS()/1000.0;

                control.set_movment(
                    robot_translation,
                    translation_velocity,
                    translation_acceleration,
                    robot_rotation,
                    angular_velocity,
                    angular_acceleration,
                    calculus_step, time 
                );
#endif

            }

            Eigen::Vector2d robot_position( 
                robot.position.getX(), robot.position.getY()
            );
            double robot_orientation = robot.orientation.getSignedValue();
            double time = TimeStamp::now().getTimeMS()/1000.0;


#ifndef CURVE_FOLLLOWING
            Eigen::Vector2d ball_position = Eigen::Vector2d(
                ball.position.getX(), ball.position.getY()
            );

            double goal_rotation = angle(ball_position - robot_position);

            double goal_rayon = .1;

            Eigen::Vector2d defender_pos = calculate_goal_position(
                ball_position, right_goal_position, left_goal_position,
                goal_rayon
            );
            
            double rayon_surface_reparation = 0.75;
            if( (defender_pos - goal_center).norm() > rayon_surface_reparation ){
                defender_pos = goal_center + Eigen::Vector2d(0.3, 0.0);
            }
            
            control.set_goal(
                defender_pos, goal_rotation
            );
#endif

            control.update( time );

            Control ctrl = control.relative_control_in_robot_frame(
                robot_position, robot_orientation
            );

            if( ctrl.velocity_translation.norm() > TRANSLATION_VELOCITY_LIMIT ){
                ctrl.velocity_translation = Eigen::Vector2d(0.0, 0.0);
                std::cerr << "WARNING : we reached the "
                    "limit translation velocity !" << std::endl;
            }

            if( std::abs( ctrl.velocity_rotation ) > ROTATION_VELOCITY_LIMIT ){
                ctrl.velocity_rotation = 0.0;
                std::cerr << "WARNING : we reached the "
                    "limit rotation velocity !" << std::endl;
            }

        #ifdef SSL_SIMU
            commander->set(
                0, true, ctrl.velocity_translation[0], ctrl.velocity_translation[1], ctrl.velocity_rotation
            );
        #else
            commander->set(
                0, true, ctrl.velocity_translation[0], -ctrl.velocity_translation[1], ctrl.velocity_rotation
            );
        #endif

            commander->flush();
        }
    }

    void AI::run()
    {
        double period = 1/100.0;    // 100 hz
        auto lastTick = TimeStamp::now();

        while (running) {
            auto now = TimeStamp::now();
            double elapsed = diffSec(lastTick, now);
            double toSleep = period - elapsed;
            if (toSleep > 0) {
                usleep(round(toSleep*1000000));
            }
            lastTick = TimeStamp::now();
            tick();
        }
    }

    void AI::stop()
    {
        running = false;
    }
}
