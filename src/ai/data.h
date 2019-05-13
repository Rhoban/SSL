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
#include <robot_behavior/robot_behavior.h>

#include "data/robot.h"
#include "data/field.h"
#include "data/ball.h"

namespace rhoban_ssl
{
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
 * This class is used to make transfer store global data accessed from many points.
 */
class GlobalDataSingleThread
{
public:
  data::Robot robots_[2][ai::Config::NB_OF_ROBOTS_BY_TEAM];
  data::Ball ball_;
  data::Field field_;

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
