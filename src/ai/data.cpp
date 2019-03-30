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

#include "data.h"

namespace rhoban_ssl
{
Data::Data(Ai::Team initial_team_color)
{
  data_from_ai_.team_color = initial_team_color;
}

SharedData::FinalControl::FinalControl()
  : hardware_is_responding(false), is_disabled_by_viewer(false), is_manually_controled_by_viewer(false)
{
}

SharedData::FinalControl::FinalControl(const FinalControl& control)
  : hardware_is_responding(control.hardware_is_responding)
  , is_disabled_by_viewer(control.is_disabled_by_viewer)
  , is_manually_controled_by_viewer(control.is_manually_controled_by_viewer)
  , control(control.control)
{
}

SharedData::SharedData() : final_control_for_robots(Ai::Constants::NB_OF_ROBOTS_BY_TEAM)
{
}

Data& Data::operator<<(const Vision::VisionData& vision_data)
{
  mutex_for_vision_data_.lock();
  vision_data_ = vision_data;
  mutex_for_vision_data_.unlock();
  return *this;
}

Data& Data::operator>>(Vision::VisionData& vision_data)
{
  mutex_for_vision_data_.lock();
  vision_data = vision_data_;
  mutex_for_vision_data_.unlock();
  return *this;
}

Data& Data::operator<<(const DataFromAi& data_from_ai)
{
  mutex_for_ai_data_.lock();
  data_from_ai_ = data_from_ai;
  mutex_for_ai_data_.unlock();
  return *this;
}

Data& Data::operator>>(DataFromAi& data_from_ai)
{
  mutex_for_ai_data_.lock();
  data_from_ai = data_from_ai_;
  mutex_for_ai_data_.unlock();
  return *this;
}

Data& Data::operator<<(const SharedData& shared_data)
{
  mutex_for_shared_data_.lock();
  shared_data_ = shared_data;
  mutex_for_shared_data_.unlock();
  return *this;
}

Data& Data::operator>>(SharedData& shared_data)
{
  mutex_for_shared_data_.lock();
  shared_data = shared_data_;
  mutex_for_shared_data_.unlock();
  return *this;
}

void Data::editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(Vision::VisionData& vision_data)> vision_data_editor)
{
  mutex_for_vision_data_.lock();
  vision_data_editor(vision_data_);
  mutex_for_vision_data_.unlock();
}

void Data::editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor)
{
  mutex_for_ai_data_.lock();
  data_from_ai_editor(data_from_ai_);
  mutex_for_ai_data_.unlock();
}

void Data::editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(SharedData& shared_data)> shared_data_editor)
{
  mutex_for_shared_data_.lock();
  shared_data_editor(shared_data_);
  mutex_for_shared_data_.unlock();
}

Data& Data::operator<<(const DataForViewer& data_for_viewer)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer_ = data_for_viewer;
  mutex_for_viewer_data_.unlock();
  return *this;
}

Data& Data::operator>>(DataForViewer& data_for_viewer)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer = data_for_viewer_;
  mutex_for_viewer_data_.unlock();
  return *this;
}

void Data::editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer_editor(data_for_viewer_);
  mutex_for_viewer_data_.unlock();
}

}  // namespace rhoban_ssl
