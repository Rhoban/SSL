#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <rhoban_utils/util.h>

#include <iostream>
#include <limits>
#include <iomanip> 

//TODO: move to rhoban_utils
#define DEBUG(message) \
    std::cerr << std::setprecision( std::numeric_limits< double >::max_digits10 ) << "# " << message << " -- " << \
    rhoban_utils::getBaseName(__FILE__) << \
    ":" << __LINE__ << std::endl

#define PLOT(message) \
    std::cout << "P " << message << std::endl

#endif
