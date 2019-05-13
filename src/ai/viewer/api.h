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

#include <libwebsockets.h>
#include <queue>

#include "ai_packet.pb.h"

#include <config.h>
#include <execution_manager.h>
#include <data.h>
#include <vision/vision_data.h>
#include <manager/factory.h>
#include <manager/manager.h>

namespace rhoban_ssl
{
namespace viewer
{
class Api
{
private:
  /**
   * @brief Constructor.
   */
  Api();

  /**
   * @brief The singleton of the class.
   */
  static Api api_singleton_;

  /**
   * @brief All packet to store and send.
   */
  std::queue<AIPacket> packets_;

  /**
   * @brief The current time
   */
  double time_;

public:
  /**
   * @brief Get the unique instance of the class.
   */
  static Api& getApi();
  /**
   * @brief Add a packet in the queue.
   */
  void addPacket(AIPacket packet);

  /**
   * @brief Get the queue of packets.
   * @return The pointer of the queue of packets.
   */
  std::queue<AIPacket>& getQueue();

  /**
   * @brief Add Field Packet.
   */
  void addFieldPacket();

  /**
   * @brief Add Entity Packet.
   */
  void addEntityPacket(ai::AiData& ai_data);

  void addListPacket(std::shared_ptr<manager::Manager> manager);

  void addElectronicsPacket();
};
}  // namespace viewer
}  // namespace rhoban_ssl
