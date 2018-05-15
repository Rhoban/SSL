#ifndef __DATA_H__
#define __DATA_H__

#include <vision/VisionData.h>
#include <mutex>
#include <AiData.h>
#include <robot_behavior/robot_behavior.h>

namespace RhobanSSL {

struct Data_from_ai {
    Ai::Team team_color;
};

struct Shared_data {
    struct Final_control {
        bool hardware_is_responding;  // Wrie access by AICommander only
        bool is_disabled_by_viewer;    // Write access for viewer only
        bool is_manually_controled_by_viewer; // Write acces for viewer only
        Control control; // Write access for viewer when is_manually_controled_by_viewer is set to true
                         // Write access for Ai
        Final_control();
        Final_control( const Final_control & control );
    };

    std::vector< Final_control > final_control_for_robots;
    
    Shared_data();
};


/**
 * This class is used to make transfer between different threads.
 */
class Data {
private: // Do not remove !

    std::mutex mutex_for_vision_data;
    Vision::VisionData vision_data;

    std::mutex mutex_for_ai_data;
    Data_from_ai data_from_ai;

    std::mutex mutex_for_shared_data;
    Shared_data shared_data;

public:
    
    Data& operator<<( const Vision::VisionData & vision_data );
    Data& operator>>( Vision::VisionData & vision_data );

    Data& operator<<( const Data_from_ai & data_from_ai );
    Data& operator>>( Data_from_ai & data_from_ai );

    Data& operator<<( const Shared_data & shared_data );
    Data& operator>>( Shared_data & shared_data );
};

}

#endif
