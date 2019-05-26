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

#include <execution_manager.h>
#include <ai.h>

namespace rhoban_ssl
{
namespace viewer
{
/**
 * @brief The ViewerCommunication task process the incomming packets
 * from viewer clients and send informations of the game and the ia.
 */
class ViewerCommunication : public Task
{
private:
  AI* ai_;

  /**
   * @brief The time when the last packets has been send to the viewer.
   */
  double last_sending_time_;

  /**
   * @brief SENDING_DELAY
   * @todo MOVE to config or modify to be a request from the viewer
   * @bug if the delay is too fast the viewer_client crash
   */
  const double SENDING_DELAY = 0.032;

public:
  ViewerCommunication(AI* ai);

  // Task interface
public:
  bool runTask();

private:
  void processIncomingPackets();
  void sendViewerPackets();

  Json::Value fieldPacket();
  Json::Value ballPacket();
  Json::Value teamsPacket();
  Json::Value refereePacket();
  Json::Value informationsPacket();
  Json::Value aiPacket();
};
}  // namespace viewer
}  // namespace rhoban_ssl
