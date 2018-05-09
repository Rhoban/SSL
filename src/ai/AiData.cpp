#include "AiData.h"
#include <assert.h>
#include <physic/movement_predicted_by_integration.h>
#include <physic/movement_with_no_prediction.h>
#include <physic/movement_on_new_frame.h>
#include <debug.h>
#include <physic/collision.h>

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
        if( this->movement ){
            assert( movement != static_cast<Movement_on_new_frame*>(this->movement)->get_original_movement() ); 
            delete this->movement;
        }
        // We change the frame according referee informatiosns
        this->movement = new Movement_on_new_frame(movement);
    }
    void Object::change_frame(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    ){
        static_cast<Movement_on_new_frame*>(movement)->set_frame(origin, v1, v2);
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
        if( vision_data.field.present ){
            static_cast<Vision::Field&>(field) = vision_data.field;
        }; 

        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                robots[team][k].set_vision_data(
                    vision_data.robots.at(team).at(k)
                );
            }
        }
        ball.set_vision_data( vision_data.ball );
        compute_table_of_collision_times();
    }

    void AiData::change_frame_for_all_objects(
        const rhoban_geometry::Point & origin,
        const Vector2d & v1, const Vector2d & v2
    ){
        team_point_of_view.set_frame( origin, v1, v2 );
        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                robots[team][k].change_frame( origin, v1, v2 );
            }
        }
        ball.change_frame( origin, v1, v2 );
    };

    void AiData::change_team_color( Ai::Team team_color ){
        this->team_color = team_color;
    }

    AiData::AiData(){
        int nb_robots = 0;
        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                robots[team][k].set_movement(
                    new Movement_with_no_prediction()
                    //new Movement_predicted_by_integration()
                );
                nb_robots++;
            }
        }
        all_robots = std::vector< std::pair<Vision::Team, Robot*> >(nb_robots);
        unsigned int i = 0;
        for( auto team : {Vision::Ally, Vision::Opponent} ){
            for( int k=0; k<Vision::Robots; k++ ){
                all_robots[i] = std::pair< Vision::Team, Robot* >(
                    team, &( robots.at(team).at(k) )
                );
                i++;
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
            
            left_post_position = Vector2d( -4.5, -0.5 );
            right_post_position = Vector2d( -4.50, 0.5 );
            goal_center = (
                left_post_position + right_post_position
            )/2;
            waiting_goal_position = (
                goal_center + Vector2d(0.0, 0.0)
            );
            // PID for translation
            p_translation = 0.05; 
            //p_translation = 0.0; 
            i_translation = .0001;
            d_translation = .0005;
            // PID for orientation
            p_orientation = 0.05;
            //p_orientation = 0.0;
            i_orientation = 0.0001;
            d_orientation = 0.0005;

            translation_velocity = 2.0;
            translation_acceleration = 1.0;
            angular_velocity = 1.0*M_PI;  
            angular_acceleration = 1*M_PI;

            calculus_step = 0.00005;
            enable_kicking = false;

            penalty_rayon = 1.0; // penalty rayon for the goalie
        #else
            DEBUG("REAL MODE ACTIVATED");
            // SSL QUALIF

            front_size = .06;

            left_post_position = Vector2d( 0., -0.29 );
            right_post_position = Vector2d( 0., 0.29 );
            goal_center = (
                left_post_position + right_post_position
            )/2;
            waiting_goal_position = (
                goal_center + Vector2d(0.3, 0.0)
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

    bool AiData::robot_is_inside_the_field( int robot_id ) const {
        const RhobanSSL::Movement & mov = robots.at(Vision::Team::Ally).at(robot_id).get_movement();
        return field.is_inside( mov.linear_position(time) );
    }

    bool AiData::robot_is_valid( int robot_id ) const {
        return (
            robots.at(Vision::Team::Ally).at(robot_id).isOk()
            and
            robot_is_inside_the_field(robot_id)
        ); 
    }

    const AiData::Collision_times_table & AiData::get_table_of_collision_times() const 
    {
        return table_of_collision_times;
    }

    void AiData::visit_all_pair_of_robots(
        std::function <
            void (
                Vision::Team robot_team_1, Robot & robot_1,
                Vision::Team robot_team_2, Robot & robot_2 
            )
        > visitor
    ){
        for( unsigned int i = 0; i < all_robots.size(); i++ ){
            for( unsigned int j = i+1; j < all_robots.size(); j++ ){
                visitor(
                    all_robots[i].first, *all_robots[i].second,
                    all_robots[j].first, *all_robots[j].second
                );
            }
        }
    }

    void AiData::compute_table_of_collision_times(){
        for( int i=0; i<all_robots.size(); i++ ){
            for( int j=i+1; j<all_robots.size(); j++ ){
                Robot & robot_1 = *all_robots[i].second;
                Robot & robot_2 = *all_robots[j].second;
                double radius_error = 0.0;
                std::pair<bool, double> collision = collision_time(
                    constants.robot_radius, 
                    robot_1.get_movement(),
                    constants.robot_radius, 
                    robot_2.get_movement(),
                    radius_error, time
                );
                if( collision.first ){
                    table_of_collision_times[ std::pair<int,int>(i,j) ] = collision.second;
                }
            }
        }
    }

/*
double Navigation_with_obstacle_avoidance::collision_time( int robot_1, int robot_2 ) const {
    std::pair<int, int> key( robot_1, robot_2 );
    assert( table_of_collision_times.find(key) != table_of_collision_times.clear() );
    return table_of_collision_times.at(key);
}
*/
 
} } //Namespace
