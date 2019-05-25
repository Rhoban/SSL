#pragma once

#include <execution_manager.h>
#include <ai.h>

namespace rhoban_ssl
{
namespace viewer
{
/**
 * @brief The ViewerCommunication task process the incomming packets
 * from viewer clients and send status informations of the game and the ia
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
   */
  const double SENDING_DELAY = 0.150;

public:
  ViewerCommunication(AI* ai);

  // Task interface
public:
  bool runTask();

private:
  void processIncomingPackets();
  void sendStatusPackets();

  Json::Value fieldStatus();
  Json::Value mobilesStatus();
  Json::Value availableManager();
  Json::Value availableRobotBehavior();
};
}  // namespace viewer
}  // namespace rhoban_ssl
