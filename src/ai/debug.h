#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <rhoban_utils/util.h>

#include <iostream>
#include <limits>

//TODO: move to rhoban_utils
#define DEBUG(message) \
    std::cerr << std::scientific << "# " << message << " -- " << \
    rhoban_utils::getBaseName(__FILE__) << \
    ":" << __LINE__ << std::endl

#define PLOT(message) \
    std::cout << std::scientific << "P " << message << std::endl

#endif
