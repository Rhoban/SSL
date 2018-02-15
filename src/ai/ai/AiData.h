#ifndef __AIDATA_H__
#define __AIDATA_H__

#include <map>
#include <geometry/Angle.hpp>
#include <timing/TimeStamp.hpp>
#include <physic/MovementSample.h>
#include <vision/VisionData.h>
#include <physic/Movement.h>

namespace RhobanSSL {
namespace Ai {

class Object {
private: 
    Vision::Object vision_data;
    RhobanSSL::Movement * movement;

public:

    Object();
    Object( const Object& object );
    Object& operator=( const Object& object );

    void set_vision_data( const Vision::Object & vision_data  );
    void set_movement( Movement * movement );

    bool isOk() const ;

    const RhobanSSL::Movement & get_movement() const; 
    virtual ~Object();
};

class Robot : public Object { };
class Ball : public Object { };

struct Field : Vision::Field { };


class AiData {
public:
    AiData();

    std::map<Vision::Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;

    void update( const Vision::VisionData vision_data);
};

} }

#endif
