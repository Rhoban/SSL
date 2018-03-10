#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <rhoban_utils/util.h>

#include <iostream>
#include <core/export_to_plot.h>

//TODO: move to rhoban_utils
#define DEBUG(message) \
    std::cerr << "# " << message << " -- " << \
    rhoban_utils::getBaseName(__FILE__) << \
    ":" << __LINE__ << std::endl

#define PLOT(message) \
    std::cout << "P " << message << std::endl

#endif
