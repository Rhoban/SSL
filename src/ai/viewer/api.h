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
// #include "ai_packet.pb.h"
// #include "game_packet.pb.h"
// #include "entity_packet.pb.h"
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
 * @brief The API for the connection with the viewer.
 */
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
  std::queue<Json::Value> ai_packets_;

  /**
   * @brief All packet receive by the viewer.
   */
  std::queue<Json::Value> viewer_packets_;

public:
  /**
   * @brief Get the unique instance of the class.
   */
  static Api& getApi();
  /**
   * @brief Add a packet in the queue.
   */
  void addAIPacket(Json::Value& packet);

  /**
   * @brief Add a packet send by the viewer in the queue.
   * @param packet_receive Packet of the viewer.
   */
  void addViewerPacket(char* viewer_packet);

  /**
   * @brief Get the queue of packets.
   * @return The pointer of the queue of packets.
   */
  std::queue<Json::Value>& getQueue();

  /**
   * @brief Generate this packet for each user connect.
   */
  void generateFirstConnectionPacket();

  /**
   * @brief Generate Mobile Packet (Robot + Ball).
   */
  void generateMobilePacket();

  /**
   * @brief Generate packet with the creation of the Manager.
   */
  void generateManagerPacket(bool json_has_changed);

  void readViewerPacket();

  void manualKick(int robot_id, double kick_power);

  void manualChipKick(int robot_id, double kick_power);

  void manualCharge(int robot_id);

  /**
   * @brief Control the robot with the Manual controller.
   */
  void manualControl(int robot_id, double xSpeed, double ySpeed, double thetaSpeed);

  void tareOdom(int robot_id, bool tare, double xPosFix, double yPosFix, double thetaFix);
};
}  // namespace viewer
}  // namespace rhoban_ssl
