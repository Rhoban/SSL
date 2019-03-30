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

#pragma once

#include <vision/vision_data.h>
#include <mutex>
#include <ai_data.h>
#include <robot_behavior/robot_behavior.h>

namespace rhoban_ssl
{
struct DataFromAi
{
  Ai::Team team_color;
};

struct DataForViewer
{
  RhobanSSLAnnotation::Annotations annotations;
};

struct SharedData
{
  struct FinalControl
  {
    bool hardware_is_responding;           // Wrie access by AICommander only
    bool is_disabled_by_viewer;            // Write access for viewer only
    bool is_manually_controled_by_viewer;  // Write acces for viewer only
    Control control;  // Write access for viewer when is_manually_controled_by_viewer is set to true
                      // Write access for Ai
    FinalControl();
    FinalControl(const FinalControl& control);
  };

  std::vector<FinalControl> final_control_for_robots;

  SharedData();
};

/**
 * This class is used to make transfer between different threads.
 */
class Data
{
private:  // Do not remove !
  std::mutex mutex_for_vision_data_;
  Vision::VisionData vision_data_;

  std::mutex mutex_for_ai_data_;
  DataFromAi data_from_ai_;

  std::mutex mutex_for_shared_data_;
  SharedData shared_data_;

  std::mutex mutex_for_viewer_data_;
  DataForViewer data_for_viewer_;

public:
  Data(Ai::Team initial_team_color);

  Data& operator<<(const Vision::VisionData& vision_data);
  Data& operator>>(Vision::VisionData& vision_data);
  void editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(Vision::VisionData& vision_data)> vision_data_editor);

  Data& operator<<(const DataFromAi& data_from_ai);
  Data& operator>>(DataFromAi& data_from_ai);
  void editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor);

  Data& operator<<(const DataForViewer& data_for_viewer);
  Data& operator>>(DataForViewer& data_for_viewer);
  void editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor);

  Data& operator<<(const SharedData& shared_data);
  Data& operator>>(SharedData& shared_data);
  void editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(SharedData& shared_data)> shared_data_editor);
};

}  // namespace RhobanSSL
