#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <rhoban_utils/util.h>

#include <iostream>
#include <limits>
#include <iomanip> 

extern double last_debug_printing;
extern bool periodic_debug_is_allowed;

void update_periodic_debug( double current_time, double period );

//TODO: move to rhoban_utils
#define DEBUG(message) \
    std::cerr << std::setprecision( std::numeric_limits< double >::max_digits10 ) << "# " << message << " -- " << \
    rhoban_utils::getBaseName(__FILE__) << \
    ":" << __LINE__ << std::endl

#define DEBUGP(message) \
    if( periodic_debug_is_allowed ){ \
        std::cerr << std::setprecision( std::numeric_limits< double >::max_digits10 ) \
            << "# " << message << " -- " << rhoban_utils::getBaseName(__FILE__) \
            << ":" << __LINE__ << std::endl; \
    }

#define PLOT(message) \
    std::cout << "P " << message << std::endl

#endif
