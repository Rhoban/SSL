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

#ifndef __MANAGER__NEW_PLAN__H__
#define __MANAGER__NEW_PLAN__H__

#include <manager/Manager.h>
#include <referee/Referee.h>

namespace RhobanSSL {
namespace Manager {

class NewPlan : public Manager {
    private:
    const Referee & referee;

    bool is_in_offensive_mode;

    bool in_defensive_free_kick;
    rhoban_geometry::Point ball_position_in_free_kick;

    unsigned int last_referee_changement;
    bool need_to_change_strategies;
    int last_nb_robot_valid;

    std::list<std::string> future_strats;

    public:

    NewPlan(
        Ai::AiData & ai_data,
        const Referee & referee
    );

    void update(double time);
    void analyse_data(double time);
    void choose_a_strategy(double time);

    virtual ~NewPlan();

};

};
};

#endif
