#include "AiData.h"
#include <assert.h>
#include <prediction/movement_prediction.h>
#include <tools/debug.h>
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

} } //Namespace
