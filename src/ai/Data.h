

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

#ifndef __DATA_H__
#define __DATA_H__

#include <vision/VisionData.h>
#include <mutex>


#include <annotations/Annotations.h>
#include "team_color.h"
#include <control/robot_control.h>

namespace RhobanSSL {

struct Data_from_ai {
  Ai::Team team_color;
};

struct Data_for_viewer {
  RhobanSSLAnnotation::Annotations annotations;
};


struct Shared_data {
  struct Final_control {
    bool hardware_is_responding;  // Wrie access by AICommander only
    bool is_disabled_by_viewer;    // Write access for viewer only
    bool is_manually_controled_by_viewer; // Write acces for viewer only
    Control control; // Write access for viewer when is_manually_controled_by_viewer is set to true
    // Write access for Ai
    Final_control();
    Final_control( const Final_control & control );
  };

  std::vector< Final_control > final_control_for_robots;

  Shared_data();
};

struct Synchro_data{

  bool synchro_is_done;
  double send_command_time; //sent by the robot behavior (tare)
  double receive_command_time; //received by the vision
  int robot_id; 
  bool flag_ready_to_receive; //to trigger time measure by vision
  bool request_synchro; //request from the manager to trigger the synchro

  double movement_thresh; //threshold for movement detection

  double cam0_offset, cam1_offset, cam2_offset, cam3_offset; //raw offsets from cameras

  Synchro_data();
};

/**
 * This class is used to make transfer between different threads.
 */
class Data {
private: // Do not remove !

  std::mutex mutex_for_vision_data;
  Vision::VisionData vision_data;

  std::mutex mutex_for_ai_data;
  Data_from_ai data_from_ai;

  std::mutex mutex_for_shared_data;
  Shared_data shared_data;

  std::mutex mutex_for_viewer_data;
  Data_for_viewer data_for_viewer;

  std::mutex mutex_for_synchro_data;
  Synchro_data data_for_synchro;
  
  
public:
  Data( Ai::Team initial_team_color );

  Data& operator<<( const Vision::VisionData & vision_data );
  Data& operator>>( Vision::VisionData & vision_data );
  void edit_vision_data( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Vision::VisionData & vision_data) > vision_data_editor
    );

  Data& operator<<( const Data_from_ai & data_from_ai );
  Data& operator>>( Data_from_ai & data_from_ai );
  void edit_data_from_ai( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Data_from_ai & data_from_ai) > data_from_ai_editor
    );

  Data& operator<<( const Data_for_viewer & data_for_viewer );
  Data& operator>>( Data_for_viewer & data_for_viewer );
  void edit_data_for_viewer( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Data_for_viewer & data_for_viewer) > data_for_viewer_editor
    );


  Data& operator<<( const Shared_data & shared_data );
  Data& operator>>( Shared_data & shared_data );
  void edit_shared_data( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Shared_data & shared_data) > shared_data_editor
    );

  void edit_synchro_data( // Use that function if you ha no choice. Prefer << and >> operator.
    std::function< void (Synchro_data & synchro_data) > synchro_data_editor 
    );

  Data& operator<<( const Synchro_data & synchro_data );
  Data& operator>>( Synchro_data & synchro_data );
  
};

}

#endif
