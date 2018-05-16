#include "Data.h"

namespace RhobanSSL {

Shared_data::Final_control::Final_control():
    hardware_is_responding(false),
    is_disabled_by_viewer(false),
    is_manually_controled_by_viewer(false)
{ }

Shared_data::Final_control::Final_control( const Final_control & control ):
    hardware_is_responding(control.hardware_is_responding),
    is_disabled_by_viewer(control.is_disabled_by_viewer),
    is_manually_controled_by_viewer(control.is_manually_controled_by_viewer),
    control(control.control)
{ }


Shared_data::Shared_data():
    final_control_for_robots(Ai::Constants::NB_OF_ROBOTS_BY_TEAM)
{
}


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

Data& Data::operator<<( const Data_from_ai & data_from_ai ){
    mutex_for_ai_data.lock();
    this->data_from_ai = data_from_ai;
    mutex_for_ai_data.unlock();
    return *this;
}

Data& Data::operator>>( Data_from_ai & data_from_ai ){
    mutex_for_ai_data.lock();
    data_from_ai = this->data_from_ai;
    mutex_for_ai_data.unlock();
    return *this;
}


Data& Data::operator<<( const Shared_data & shared_data ){
    mutex_for_shared_data.lock();
    this->shared_data = shared_data;
    mutex_for_shared_data.unlock();
    return *this;
}

Data& Data::operator>>( Shared_data & shared_data ){
    mutex_for_shared_data.lock();
    shared_data = this->shared_data;
    mutex_for_shared_data.unlock();
    return *this;
}

} //Namespace
