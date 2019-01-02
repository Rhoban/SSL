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

#ifndef __RULES__H__
#define __RULES__H__


#include <manager/manager_with_game_state.h>
#include <type_traits>

namespace RhobanSSL
{
namespace Manager
{

template< typename MANAGER,
          typename = typename std::enable_if<std::is_base_of<ManagerWithGameState, MANAGER>::value>::type>
class Rules : public MANAGER
{

  public:
      Rules(Ai::AiData & ai_data, const GameState & game_state):MANAGER(ai_data, game_state)
      {
            DEBUG("\n CONSTRUCT \n");
      }

        virtual void start_stop(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_stop();
                DEBUG("\n coucou START STOP\n");
	}

	virtual void start_running(){
                MANAGER::set_ball_avoidance_for_all_robots(false);
                MANAGER::start_running();
                DEBUG("\n coucou RUNNING \n");
	}
	virtual void start_halt(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_halt();
                DEBUG("\n coucou HALT\n");
	}

	virtual void start_direct_kick_ally(){
                MANAGER::set_ball_avoidance_for_all_robots(false);
                MANAGER::start_direct_kick_ally();
	}
	virtual void start_direct_kick_opponent(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_direct_kick_opponent();
	}

	virtual void start_indirect_kick_ally(){
                MANAGER::set_ball_avoidance_for_all_robots(false);
                MANAGER::start_indirect_kick_ally();
	}
	virtual void start_indirect_kick_opponent(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_indirect_kick_opponent();
	}

	virtual void start_prepare_kickoff_ally(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_prepare_kickoff_ally();
	}
	virtual void start_prepare_kickoff_opponent(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_prepare_kickoff_opponent();
	}

	virtual void start_kickoff_ally(){
                MANAGER::set_ball_avoidance_for_all_robots(false);
                MANAGER::start_kickoff_ally();
	}
	virtual void start_kickoff_opponent(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_kickoff_opponent();
	}

	virtual void start_penalty_ally(){
                MANAGER::set_ball_avoidance_for_all_robots(false);
                MANAGER::start_penalty_ally();
	}
	virtual void start_penalty_opponent(){
                MANAGER::set_ball_avoidance_for_all_robots(true);
                MANAGER::start_penalty_opponent();
	}

	//Continue

	virtual void continue_stop(){
	}

	virtual void continue_running(){
                MANAGER::continue_running();
	}
	virtual void continue_halt(){
                MANAGER::continue_halt();
	}

	virtual void continue_direct_kick_ally(){
                MANAGER::continue_direct_kick_ally();
	}
	virtual void continue_direct_kick_opponent(){
                MANAGER::continue_direct_kick_opponent();
	}

	virtual void continue_indirect_kick_ally(){
                MANAGER::continue_indirect_kick_ally();
	}
	virtual void continue_indirect_kick_opponent(){
                MANAGER::continue_indirect_kick_opponent();
	}

	virtual void continue_prepare_kickoff_ally(){
                MANAGER::continue_prepare_kickoff_ally();
	}
	virtual void continue_prepare_kickoff_opponent(){
                MANAGER::continue_prepare_kickoff_opponent();
	}

	virtual void continue_kickoff_ally(){
                MANAGER::continue_kickoff_ally();
	}
	virtual void continue_kickoff_opponent(){
                MANAGER::continue_kickoff_opponent();
	}

	virtual void continue_penalty_ally(){
                MANAGER::continue_penalty_ally();
	}
	virtual void continue_penalty_opponent(){
                MANAGER::continue_penalty_opponent();
	}
};

} // namespace Manager
} // namespace RhobanSSL
#endif
