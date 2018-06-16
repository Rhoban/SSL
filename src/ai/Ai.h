/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

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

#ifndef __AI__H__
#define __AI__H__

#include <com/AICommander.h>
#include <vision/AIVisionClient.h>
#include <robot_behavior/robot_behavior.h>
#include "AiData.h"
#include <referee/Referee.h>
#include <core/machine_state.h>
#include <manager/Manager.h>
#include <annotations/Annotations.h>

namespace RhobanSSL
{

    class AI
    {
    private:
        std::string team_name;
        Ai::Team default_team;
    public:

        AI(
            std::string manager_name,
            std::string team_name,
            Ai::Team default_team,
            Data & data,
            AICommander *commander,
            const std::string & config_path,
            bool is_in_simulation
        );

        void run();
        void stop();

        std::vector<std::string> getAvailableManagers();
        void setManager(std::string manager);
        std::shared_ptr<Manager::Manager> getManager() const;
        std::shared_ptr<Manager::Manager> getManualManager();

        Referee &getReferee();

        double getCurrentTime();

    protected:
        bool running;

        Vision::VisionData visionData;
        Ai::AiData ai_data;

        bool enable_kicking;

        AICommander *commander;

        std::map<
            int,
            std::shared_ptr<Robot_behavior::RobotBehavior>
        > robot_behaviors;

        void init_robot_behaviors();
        void update_robots( );
        double current_time;
        double current_dt;

        Shared_data shared_data;

        Data & data;
        Referee referee;
        std::string manager_name;
        std::shared_ptr<Manager::Manager> strategy_manager;
        std::shared_ptr<Manager::Manager> manual_manager;

        Control update_robot(
            Robot_behavior::RobotBehavior & robot_behavior,
            double time, Ai::Robot & robot, Ai::Ball & ball
        );
        void update_electronic_informations();
        void print_electronic_info();

        void send_control( int robot_id, const Control & control );
        void prepare_to_send_control( int robot_id, Control & control );

        void limits_velocity( Control & ctrl ) const ;
        void check_time_is_coherent() const;

        void share_data();
        void prevent_collision( int robot_id, Control & ctrl );
        RhobanSSLAnnotation::Annotations get_robot_behavior_annotations() const;
        public:
         
        void get_annotations( RhobanSSLAnnotation::Annotations & annotations ) const;

    };
};

#endif
