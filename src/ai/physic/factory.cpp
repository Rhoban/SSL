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
