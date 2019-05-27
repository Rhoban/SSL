/*
    This file is part of SSL.

    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

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

#ifndef __MANAGER__PLANVESCHAMBRES__H__
#define __MANAGER__PLANVESCHAMBRES__H__

#include <manager/Manager.h>
#include <referee/game_state.h>

namespace rhoban_ssl
{
namespace Manager
{
class PlanVeschambres : public Manager
{
private:
  const Referee& referee;

  // penalty
  std::vector<std::list<std::string> > penalty_strats;
  // goale
  std::vector<std::list<std::string> > goalie_strats;
  // kick
  std::vector<std::list<std::string> > kick_strats;
  // kick_strats_indirect
  std::vector<std::list<std::string> > kick_strats_indirect;
  // offensiv
  std::vector<std::list<std::string> > offensive_strats;
  // defensive
  std::vector<std::list<std::string> > defensive_strats;

  std::vector<std::list<std::string> > stop_strats;

  std::string strategy_applied = "";

  bool hold_ball_position = true;
  rhoban_geometry::Point ball_last_position;

  unsigned int last_referee_changement;

  std::list<std::string> future_strats;

public:
  PlanVeschambres(ai::AiData& ai, const GameState& game_state);

  void update(double time);
  void analyse_data(double time);
  void choose_a_strategy(double time);

  virtual ~PlanVeschambres();
};

};  // namespace Manager
};  // namespace rhoban_ssl

#endif
