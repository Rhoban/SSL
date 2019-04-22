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
//#include <ai_data.h>
#include <robot_behavior/robot_behavior.h>

namespace rhoban_ssl
{
struct DataFromAi
{
  ai::Team team_color;
};

struct DataForViewer
{
  rhoban_ssl::annotations::Annotations annotations;
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
class GlobalData
{
private:  // Do not remove !
  std::mutex mutex_for_vision_data_;
  vision::VisionData vision_data_;

  std::mutex mutex_for_ai_data_;
  DataFromAi data_from_ai_;

  std::mutex mutex_for_shared_data_;
  SharedData shared_data_;

  std::mutex mutex_for_viewer_data_;
  DataForViewer data_for_viewer_;

public:
  GlobalData(ai::Team initial_team_color);

  GlobalData& operator<<(const vision::VisionData& vision_data);
  GlobalData& operator>>(vision::VisionData& vision_data);
  void editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(vision::VisionData& vision_data)> vision_data_editor);

  GlobalData& operator<<(const DataFromAi& data_from_ai);
  GlobalData& operator>>(DataFromAi& data_from_ai);
  void editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor);

  GlobalData& operator<<(const DataForViewer& data_for_viewer);
  GlobalData& operator>>(DataForViewer& data_for_viewer);
  void editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor);

  GlobalData& operator<<(const SharedData& shared_data);
  GlobalData& operator>>(SharedData& shared_data);
  void editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
      std::function<void(SharedData& shared_data)> shared_data_editor);
};

/**
 * This class is used to make transfer store global data accessed from many points.
 */
class GlobalDataSingleThread
{
public:
  vision::VisionDataSingleThread vision_data_;
  DataFromAi data_from_ai_;
  SharedData shared_data_;
  DataForViewer data_for_viewer_;

private:
  GlobalDataSingleThread(ai::Team initial_team_color);

public:
  static GlobalDataSingleThread singleton_;
  void setTeam(ai::Team team_color);

  /*  GlobalDataSingleThread& operator<<(const vision::VisionDataSingleThread& vision_data);
    GlobalDataSingleThread& operator>>(vision::VisionDataSingleThread& vision_data);
    void editVisionData(  // Use that function if you ha no choice. Prefer << and >> operator.
        std::function<void(vision::VisionDataSingleThread& vision_data)> vision_data_editor);

    GlobalDataSingleThread& operator<<(const DataFromAi& data_from_ai);
    GlobalDataSingleThread& operator>>(DataFromAi& data_from_ai);
    void editDataFromAi(  // Use that function if you ha no choice. Prefer << and >> operator.
        std::function<void(DataFromAi& data_from_ai)> data_from_ai_editor);

    GlobalDataSingleThread& operator<<(const DataForViewer& data_for_viewer);
    GlobalDataSingleThread& operator>>(DataForViewer& data_for_viewer);
    void editDataForViewer(  // Use that function if you ha no choice. Prefer << and >> operator.
        std::function<void(DataForViewer& data_for_viewer)> data_for_viewer_editor);

    GlobalDataSingleThread& operator<<(const SharedData& shared_data);
    GlobalDataSingleThread& operator>>(SharedData& shared_data);
    void editSharedData(  // Use that function if you ha no choice. Prefer << and >> operator.
        std::function<void(SharedData& shared_data)> shared_data_editor);
        */
};

}  // namespace rhoban_ssl
