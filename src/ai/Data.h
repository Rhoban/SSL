#ifndef __DATA_H__
#define __DATA_H__

#include <vision/VisionData.h>
#include <mutex>

namespace RhobanSSL {

/**
 * This class is used to make transfer between different threads.
 */
class Data {
private: // Do not remove !

    std::mutex mutex_for_vision_data;
    Vision::VisionData vision_data;

public:

    Data& operator<<( const Vision::VisionData & vision_data );
    Data& operator>>( Vision::VisionData & vision_data );
};

}

#endif
