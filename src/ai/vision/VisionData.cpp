#include "VisionData.h"
#include <assert.h>
#include <debug.h>

namespace RhobanSSL {
namespace Vision {

void Object::update(
    double time, const Point & linear_position
){
    update( time, linear_position, movement[0].angular_position );
}

void Object::update(
    double time, const Point & linear_position, 
    const Angle & angular_position 
){
    ContinuousAngle angle( movement[0].angular_position );
    angle.set_to_nearest( angular_position );
    update( time, linear_position, angle );
}

void Object::update(
    double time, const Point & linear_position, 
    const ContinuousAngle & angular_position 
){
    if( time <= movement.time(0) ){
        //TODO
        //DEBUG("TODO");
        return;
    }
    lastUpdate = Utils::Timing::TimeStamp::now();
    present = true;

    movement.insert(
        PositionSample(time,linear_position, angular_position)
    );
}


double Object::age() const {
    return diffSec(lastUpdate, Utils::Timing::TimeStamp::now());
}

bool Object::isOk() const {
    return present && age() < 1;
}

Object::Object(): movement(history_size) {
    for( int i=0; i<history_size; i++ ){
        movement[i].time = -i;
    }
}


VisionData::VisionData(){
    field.present = false;

    for (auto team : {Ally, Opponent}) {
        for (int k=0; k<Robots; k++) {
            robots[team][k].present = false;
            robots[team][k].id = k;
        }
    }
}

void Object::checkAssert( double time ) const {
    assert(
        not(present) or ( 
            movement.time(0) > movement.time(1) 
            and
            movement.time(1) > movement.time(2) 
        ) 
    );
//    assert(
//        not(present) or ( time > movement.time(0) ) 
//    );
}


void VisionData::checkAssert( double time ) const {
    for (auto team : {Ally, Opponent}) {
        for (int k=0; k<Robots; k++) {
            robots.at(team).at(k).checkAssert( time );
        }
    }
    ball.checkAssert( time );
}

double VisionData::older_time() const {
    double older = ball.movement.time( 0 );
    for (auto team : {Ally, Opponent}) {
        for (int k=0; k<Robots; k++) {
            double t = robots.at(team).at(k).movement.time(0);
            if( t != 0 ){
                older = std::min( older, t );
            }
        }
    }
    return older;
};

std::ostream& operator<<(std::ostream& out, const RhobanSSL::Vision::VisionData& vision){
    for (auto team : {RhobanSSL::Vision::Ally, RhobanSSL::Vision::Opponent}) {
        out << team << " : " << std::endl;
        for (int k=0; k<RhobanSSL::Vision::Robots; k++) {
            out << "robot " << k << std::endl;
            out << vision.robots.at(team).at(k);
        }
    }
    out << "ball : " << std::endl;
    out << vision.ball << std::endl;
    
    return out;
}

std::ostream& operator<<(std::ostream& out, const RhobanSSL::Vision::Object& object){
    out << " id : " << object.id << std::endl;
    out << " present : " << object.present << std::endl;
    out << " lastUpdate : " << object.lastUpdate.getTimeMS()/1000.0 << std::endl;
    out << " movement : " << object.movement << std::endl;
    return out;
}


} } //Namespace


