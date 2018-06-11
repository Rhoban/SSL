/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "factory.h"

#include <AiData.h>
#include <physic/movement_predicted_by_integration.h>
#include <physic/movement_with_no_prediction.h>
#include <physic/movement_on_new_frame.h>
#include <physic/movement_with_temporal_shift.h>

namespace RhobanSSL {
namespace physic {

Movement* Factory::movement(Ai::AiData & ai_data){
    return new Movement_with_temporal_shift(
        new Movement_with_no_prediction()
        //new Movement_predicted_by_integration()
        , [&ai_data](){ return ai_data.time_shift_with_vision; }
    );
}

Movement* Factory::robot_movement(Ai::AiData & ai_data){
    return Factory::movement(ai_data);
}

Movement* Factory::ball_movement(Ai::AiData & ai_data){
    return Factory::movement(ai_data);
}


};
};
