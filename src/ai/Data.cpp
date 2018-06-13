/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Data.h"

namespace RhobanSSL {

Data::Data( Ai::Team initial_team_color ){
    data_from_ai.team_color = initial_team_color;
}

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

void Data::edit_vision_data( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Vision::VisionData & vision_data) > vision_data_editor 
){
    mutex_for_vision_data.lock();
    vision_data_editor(vision_data);
    mutex_for_vision_data.unlock();
}

void Data::edit_data_from_ai( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Data_from_ai & data_from_ai) > data_from_ai_editor 
){
    mutex_for_ai_data.lock();
    data_from_ai_editor(data_from_ai);
    mutex_for_ai_data.unlock();
}

void Data::edit_shared_data( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Shared_data & shared_data) > shared_data_editor 
){
    mutex_for_shared_data.lock();
    shared_data_editor(shared_data);
    mutex_for_shared_data.unlock();
}



Data& Data::operator<<( const Data_for_viewer & data_for_viewer ){
    mutex_for_viewer_data.lock();
    this->data_for_viewer = data_for_viewer;
    mutex_for_viewer_data.unlock();
    return *this;
}

Data& Data::operator>>( Data_for_viewer & data_for_viewer ){
    mutex_for_viewer_data.lock();
    data_for_viewer = this->data_for_viewer;
    mutex_for_viewer_data.unlock();
    return *this;
}

void Data::edit_data_for_viewer( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Data_for_viewer & data_for_viewer) > data_for_viewer_editor
){
    mutex_for_viewer_data.lock();
    data_for_viewer_editor(data_for_viewer);
    mutex_for_viewer_data.unlock();
}


} //Namespace
