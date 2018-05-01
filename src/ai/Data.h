#ifndef __DATA_H__
#define __DATA_H__

#include <vision/VisionData.h>
#include <mutex>
#include <AiData.h>

namespace RhobanSSL {

struct Data_from_ai {
    Ai::Team team_color;
};

/**
 * This class is used to make transfer between different threads.
 */
class Data {
private: // Do not remove !

    std::mutex mutex_for_vision_data;
    Vision::VisionData vision_data;

    std::mutex mutex_for_ai_data;
    Data_from_ai ai_data;

public:

    Data& operator<<( const Vision::VisionData & vision_data );
    Data& operator>>( Vision::VisionData & vision_data );

    Data& operator<<( const Data_from_ai & data_from_ai );
    Data& operator>>( Data_from_ai & data_from_ai );
};

}

#endif
