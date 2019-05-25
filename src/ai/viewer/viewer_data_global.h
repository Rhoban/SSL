/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

// STL
#include <queue>
#include <iostream>

// Packet Protobuf.
#include "ai_packet.pb.h"
#include "game_packet.pb.h"
#include "entity_packet.pb.h"
#include <json/json.h>

// Internal files
#include <config.h>
#include <execution_manager.h>
#include <data.h>
#include <vision/vision_data.h>
#include <manager/factory.h>
#include <manager/manager.h>
#include <com/ai_commander.h>

namespace rhoban_ssl
{
namespace viewer
{
/**
 * @brief The ViewerDataGlobal class contains all the data exchanged between the monitor and the ia
 * in the form of json packets.
 */
class ViewerDataGlobal
{
private:
  /**
   * @brief Constructor.
   */
  ViewerDataGlobal();

  /**
   * @brief The singleton of the class.
   */
  static ViewerDataGlobal instance_;

public:
  /**
   * @brief Get the unique instance of the class.
   */
  static ViewerDataGlobal& get();

  /**
   * @brief Store all packets that will be send to clients.
   */
  std::queue<Json::Value> packets_to_send;

  /**
   * @brief All packet received from the viewer.
   */
  std::queue<Json::Value> received_packets;

  /**
   * @brief Add a packet that will be send to viewer clients
   * in the queue.
   */
  void addPacket(const Json::Value& packet);

  /**
   * @brief Parse and add store a packet send by the viewer.
   * @param packet_received in char*.
   */
  void parseAndStorePacketFromClient(char* packet_received);
};
}  // namespace viewer
}  // namespace rhoban_ssl
