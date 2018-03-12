#include "AiData.h"
#include <assert.h>
#include <physic/movement_predicted_by_integration.h>
#include <physic/movement_with_no_prediction.h>
#include <debug.h>
namespace RhobanSSL {
namespace Ai {

    Object::Object( const Object& object ):
        vision_data(object.vision_data), movement( object.movement->clone() )
    { }

    Object& Object::operator=( const Object& object ){
        this->vision_data = vision_data;
        this->movement = object.movement->clone();
        return *this;
    }


    void Object::set_vision_data( const Vision::Object & vision_data  ){
        this->vision_data = vision_data;
        this->movement->set_sample( this->vision_data.movement );
    }
    void Object::set_movement( Movement * movement ){
        assert( movement != this->movement ); 
        if( this->movement ){
            delete this->movement;
        }
        this->movement = movement;
    }

    const RhobanSSL::Movement & Object::get_movement() const {
        return *movement;
    }    

    Object::~Object(){
        delete movement;
    }

    Object::Object():
        movement(0)
    { }

    void AiData::update( const Vision::VisionData vision_data ){
        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                robots[team][k].set_vision_data(
                    vision_data.robots.at(team).at(k)
                );
            }
        }
        ball.set_vision_data( vision_data.ball );
    }

    AiData::AiData(){
        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                robots[team][k].set_movement(
                    new Movement_with_no_prediction()
                    //new Movement_predicted_by_integration()
                );
            }
        }
        ball.set_movement(
            new Movement_with_no_prediction()
            //new Movement_predicted_by_integration()
        );
    }

    bool Ai::Object::isOk() const {
        //TODO
        return vision_data.isOk(); 
    }

    void Constants::init(){
        robot_radius = 0.09;
        radius_ball = 0.04275/2.0;
        translation_velocity_limit = TRANSLATION_VELOCITY_LIMIT;
        rotation_velocity_limit = ROTATION_VELOCITY_LIMIT;
        #ifdef SSL_SIMU
            DEBUG("SIMULATION MODE ACTIVATED");
            // SSL SIMUL
            
            front_size = .06;
            
            left_post_position = Eigen::Vector2d( -4.5, -0.5 );
            right_post_position = Eigen::Vector2d( -4.50, 0.5 );
            goal_center = (
                left_post_position + right_post_position
            )/2;
            waiting_goal_position = (
                goal_center + Eigen::Vector2d(0.0, 0.0)
            );
            // PID for translation
            p_translation = 0.01; 
            i_translation = .0;
            d_translation = .0;
            // PID for orientation
            p_orientation = 0.01;
            i_orientation = 0.0;
            d_orientation = 0.0;

            translation_velocity = 2.0;
            translation_acceleration = 0.5;
            angular_velocity = 1.0*M_PI;  
            angular_acceleration = 1*M_PI;

            calculus_step = 0.001;
            enable_kicking = false;

            penalty_rayon = 1.0; // penalty rayon for the goalie
        #else
            DEBUG("REAL MODE ACTIVATED");
            // SSL QUALIF

            front_size = .06;

            left_post_position = Eigen::Vector2d( 0., -0.29 );
            right_post_position = Eigen::Vector2d( 0., 0.29 );
            goal_center = (
                left_post_position + right_post_position
            )/2;
            waiting_goal_position = (
                goal_center + Eigen::Vector2d(0.3, 0.0)
            );
            // PID for translation
            p_translation = 0.02; 
            i_translation = .001;
            d_translation = .0;
            // PID for orientation
            p_orientation = 0.02;
            i_orientation = 0.001;
            d_orientation = 0.0;

            translation_velocity = 0.5;
            translation_acceleration = 1.;
            angular_velocity = 1.0;  
            angular_acceleration = 5.;
            calculus_step = 0.001;
            enable_kicking = true;

            penalty_rayon = 10.0; // For the goalie
        #endif
    }
 
} } //Namespace
