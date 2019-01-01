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


namespace RhobanSSL
{
namespace Manager
{

template< typename MANAGER>
class Rules : public ManagerWithGameState
{
  private:
        MANAGER *manager;

  public:
      Rules(Ai::AiData & ai_data, const GameState & game_state)
          :
            ManagerWithGameState(ai_data, game_state),
            manager{ new MANAGER(ai_data, game_state) }
      {}

	virtual void start_stop(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_stop();
	}

	virtual void start_running(){
		set_ball_avoidance_for_all_robots(false);
                manager->start_running();
	}
	virtual void start_halt(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_halt();
	}

	virtual void start_direct_kick_ally(){
		set_ball_avoidance_for_all_robots(false);
                manager->start_direct_kick_ally();
	}
	virtual void start_direct_kick_opponent(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_direct_kick_opponent();
	}

	virtual void start_indirect_kick_ally(){
		set_ball_avoidance_for_all_robots(false);
                manager->start_indirect_kick_ally();
	}
	virtual void start_indirect_kick_opponent(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_indirect_kick_opponent();
	}

	virtual void start_prepare_kickoff_ally(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_prepare_kickoff_ally();
	}
	virtual void start_prepare_kickoff_opponent(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_prepare_kickoff_opponent();
	}

	virtual void start_kickoff_ally(){
		set_ball_avoidance_for_all_robots(false);
                manager->start_kickoff_ally();
	}
	virtual void start_kickoff_opponent(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_kickoff_opponent();
	}

	virtual void start_penalty_ally(){
		set_ball_avoidance_for_all_robots(false);
                manager->start_penalty_ally();
	}
	virtual void start_penalty_opponent(){
		set_ball_avoidance_for_all_robots(true);
                manager->start_penalty_opponent();
	}

	//Continue

	virtual void continue_stop(){
	}

	virtual void continue_running(){
                manager->continue_running();
	}
	virtual void continue_halt(){
                manager->continue_halt();
	}

	virtual void continue_direct_kick_ally(){
                manager->continue_direct_kick_ally();
	}
	virtual void continue_direct_kick_opponent(){
                manager->continue_direct_kick_opponent();
	}

	virtual void continue_indirect_kick_ally(){
                manager->continue_indirect_kick_ally();
	}
	virtual void continue_indirect_kick_opponent(){
                manager->continue_indirect_kick_opponent();
	}

	virtual void continue_prepare_kickoff_ally(){
                manager->continue_prepare_kickoff_ally();
	}
	virtual void continue_prepare_kickoff_opponent(){
                manager->continue_prepare_kickoff_opponent();
	}

	virtual void continue_kickoff_ally(){
                manager->continue_kickoff_ally();
	}
	virtual void continue_kickoff_opponent(){
                manager->continue_kickoff_opponent();
	}

	virtual void continue_penalty_ally(){
                manager->continue_penalty_ally();
	}
	virtual void continue_penalty_opponent(){
                manager->continue_penalty_opponent();
	}

	virtual ~Rules()
	{
                delete manager;
	}
};

} // namespace Manager
} // namespace RhobanSSL
#endif
