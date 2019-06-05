#pragma once

#include "manager.h"
#include "referee/game_state.h"

namespace rhoban_ssl
{
namespace manager
{
class ManagerWithGameState : public Manager
{
  unsigned int last_change_stamp_;

public:
  ManagerWithGameState(std::string name);

  void update();
  void analyseData();
  void chooseAStrategy();

  // Begin of a new state
  virtual void startStop() = 0;
  virtual void startRunning() = 0;
  virtual void startHalt() = 0;

  virtual void startDirectKickAlly() = 0;
  virtual void startDirectKickOpponent() = 0;

  virtual void startIndirectKickAlly() = 0;
  virtual void startIndirectKickOpponent() = 0;

  virtual void startPrepareKickoffAlly() = 0;
  virtual void startPrepareKickoffOpponent() = 0;

  virtual void startKickoffAlly() = 0;
  virtual void startKickoffOpponent() = 0;

  virtual void startPenaltyAlly() = 0;
  virtual void startPenaltyOpponent() = 0;

  // During a state
  virtual void continueStop() = 0;
  virtual void continueRunning() = 0;
  virtual void continueHalt() = 0;

  virtual void continueDirectKickAlly() = 0;
  virtual void continueDirectKickOpponent() = 0;

  virtual void continueIndirectKickAlly() = 0;
  virtual void continueIndirectKickOpponent() = 0;

  virtual void continuePrepareKickoffAlly() = 0;
  virtual void continuePrepareKickoffOpponent() = 0;

  virtual void continueKickoffAlly() = 0;
  virtual void continueKickoffOpponent() = 0;

  virtual void continuePenaltyAlly() = 0;
  virtual void continuePenaltyOpponent() = 0;

  virtual ~ManagerWithGameState();
};

};  // namespace manager
};  // namespace rhoban_ssl
