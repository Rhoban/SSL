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
GlobalData::GlobalData(ai::Team initial_team_color)
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

SharedData::SharedData() : final_control_for_robots(ai::Config::NB_OF_ROBOTS_BY_TEAM)
{
}

GlobalData& GlobalData::operator<<(const vision::VisionData& vision_data)
{
  mutex_for_vision_data_.lock();
  // vision_data_ = vision_data;
  assert(false && "should not be called!");
  mutex_for_vision_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator>>(vision::VisionData& vision_data)
{
  mutex_for_vision_data_.lock();
  // vision_data = vision_data_;
  assert(false && "should not be called!");
  mutex_for_vision_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator<<(const DataFromAi& data_from_ai)
{
  mutex_for_ai_data_.lock();
  data_from_ai_ = data_from_ai;
  mutex_for_ai_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator>>(DataFromAi& data_from_ai)
{
  mutex_for_ai_data_.lock();
  data_from_ai = data_from_ai_;
  mutex_for_ai_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator<<(const SharedData& shared_data)
{
  mutex_for_shared_data_.lock();
  shared_data_ = shared_data;
  mutex_for_shared_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator>>(SharedData& shared_data)
{
  mutex_for_shared_data_.lock();
  shared_data = shared_data_;
  mutex_for_shared_data_.unlock();
  return *this;
}

void GlobalData::editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(vision::VisionData& vision_data)> vision_data_editor)
{
  mutex_for_vision_data_.lock();
  vision_data_editor(vision_data_);
  mutex_for_vision_data_.unlock();
}

void GlobalData::editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor)
{
  mutex_for_ai_data_.lock();
  data_from_ai_editor(data_from_ai_);
  mutex_for_ai_data_.unlock();
}

void GlobalData::editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(SharedData& shared_data)> shared_data_editor)
{
  mutex_for_shared_data_.lock();
  shared_data_editor(shared_data_);
  mutex_for_shared_data_.unlock();
}

GlobalData& GlobalData::operator<<(const DataForViewer& data_for_viewer)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer_ = data_for_viewer;
  mutex_for_viewer_data_.unlock();
  return *this;
}

GlobalData& GlobalData::operator>>(DataForViewer& data_for_viewer)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer = data_for_viewer_;
  mutex_for_viewer_data_.unlock();
  return *this;
}

void GlobalData::editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor)
{
  mutex_for_viewer_data_.lock();
  data_for_viewer_editor(data_for_viewer_);
  mutex_for_viewer_data_.unlock();
}

///////////////////////////////////////////////////////////////////////

GlobalDataSingleThread::GlobalDataSingleThread(ai::Team initial_team_color)
{
  data_from_ai_.team_color = initial_team_color;
}

void GlobalDataSingleThread::setTeam(ai::Team team_color)
{
  data_from_ai_.team_color = team_color;
}

GlobalDataSingleThread GlobalDataSingleThread::singleton_(ai::Team::Unknown);

/*

GlobalDataSingleThread& GlobalDataSingleThread::operator<<(const vision::VisionDataSingleThread& vision_data)
{
  //  mutex_for_vision_data_.lock();
  vision_data_ = vision_data;
  //  mutex_for_vision_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator>>(vision::VisionDataSingleThread& vision_data)
{
  //  mutex_for_vision_data_.lock();
  vision_data = vision_data_;
  //  mutex_for_vision_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator<<(const DataFromAi& data_from_ai)
{
  //  mutex_for_ai_data_.lock();
  data_from_ai_ = data_from_ai;
  //  mutex_for_ai_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator>>(DataFromAi& data_from_ai)
{
  //  mutex_for_ai_data_.lock();
  data_from_ai = data_from_ai_;
  //  mutex_for_ai_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator<<(const SharedData& shared_data)
{
  // mutex_for_shared_data_.lock();
  shared_data_ = shared_data;
  // mutex_for_shared_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator>>(SharedData& shared_data)
{
  // mutex_for_shared_data_.lock();
  shared_data = shared_data_;
  // mutex_for_shared_data_.unlock();
  return *this;
}

void GlobalDataSingleThread::editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(vision::VisionDataSingleThread& vision_data)> vision_data_editor)
{
  // mutex_for_vision_data_.lock();
  vision_data_editor(vision_data_);
  // mutex_for_vision_data_.unlock();
}

void GlobalDataSingleThread::editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor)
{
  // mutex_for_ai_data_.lock();
  data_from_ai_editor(data_from_ai_);
  // mutex_for_ai_data_.unlock();
}

void GlobalDataSingleThread::editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(SharedData& shared_data)> shared_data_editor)
{
  // mutex_for_shared_data_.lock();
  shared_data_editor(shared_data_);
  // mutex_for_shared_data_.unlock();
}

GlobalDataSingleThread& GlobalDataSingleThread::operator<<(const DataForViewer& data_for_viewer)
{
  // mutex_for_viewer_data_.lock();
  data_for_viewer_ = data_for_viewer;
  // mutex_for_viewer_data_.unlock();
  return *this;
}

GlobalDataSingleThread& GlobalDataSingleThread::operator>>(DataForViewer& data_for_viewer)
{
  // mutex_for_viewer_data_.lock();
  data_for_viewer = data_for_viewer_;
  // mutex_for_viewer_data_.unlock();
  return *this;
}

void GlobalDataSingleThread::editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
    std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor)
{
  // mutex_for_viewer_data_.lock();
  data_for_viewer_editor(data_for_viewer_);
  // mutex_for_viewer_data_.unlock();
}
*/

}  // namespace rhoban_ssl
