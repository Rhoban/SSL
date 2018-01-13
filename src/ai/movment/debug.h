#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <iostream>

#define DEBUG(message) \
    std::cerr << "DEBUG : " << message << \
    " -- " <<  __LINE__ << ", " << __FUNCTION__ << ", " << __FILE__ << \
    std::endl

#endif
