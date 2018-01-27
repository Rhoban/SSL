#include "Data.h"

namespace RhobanSSL {

Data& Data::operator<<( const Vision::VisionData & vision_data ){
    mutex_for_vision_data.lock();
    this->vision_data = vision_data;
    mutex_for_vision_data.unlock();
    return *this;
}

Data& Data::operator>>( Vision::VisionData & vision_data ){
    mutex_for_vision_data.lock();
    vision_data = this->vision_data;
    mutex_for_vision_data.unlock();
    return *this;
}



} //Namespace
