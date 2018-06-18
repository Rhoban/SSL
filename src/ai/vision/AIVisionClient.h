/*
  This file is part of SSL.

  Copyright 2018 Grégoire Passault (gregoire.passault@u-bordeaux.fr)
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

#include <VisionClient.h>
#include "VisionData.h"
#include <Data.h>
#include <AiData.h>
#include "factory.h"
#include "client_config.h"
#include "robot_position_filter.h" 

namespace RhobanSSL
{
class AIVisionClient : public VisionClient
{
public:

  AIVisionClient(
    Data& shared_data, Ai::Team myTeam, bool simulation = false,
    Vision::Part_of_the_field part_of_the_field_used = Vision::Part_of_the_field::ALL_FIELD
    );

  AIVisionClient(
    Data& shared_data, Ai::Team myTeam, bool simulation,
    std::string addr, std::string port, std::string sim_port,
    Vision::Part_of_the_field part_of_the_field_used = Vision::Part_of_the_field::ALL_FIELD
    );
  
  void setRobotPos(Ai::Team team, int id, double x, double y, double orientation);

protected:
  virtual void packetReceived();

  Data & shared_data;

  Vision::Part_of_the_field part_of_the_field_used;

  std::map<
    int,
    SSL_DetectionFrame
    > camera_detections;
  
    std::map<
        int, // CMAERA ID
        std::pair<
            bool, //camera have found a ball
            rhoban_geometry::Point // detecte ball
        >
    > ball_camera_detections;


  void updateRobotInformation(
    const SSL_DetectionFrame & detection,
    const SSL_DetectionRobot & robot, bool ally,
    Ai::Team team_color
    );
    
private:

  Vision::VisionData oldVisionData;
  Vision::VisionData visionData;
  Ai::Team myTeam;
  std::map<int, SSL_DetectionFrame> historic;


    rhoban_geometry::Point
    average_filter(
        const rhoban_geometry::Point & new_ball,
        std::map<
            int, // CMAERA ID
            std::pair<
                bool, //camera have found a ball
                rhoban_geometry::Point // detecte ball
            >
        > & ball_camera_detections,
        Vision::Part_of_the_field part_of_the_field_used
    );


};
}
