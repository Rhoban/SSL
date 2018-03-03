#ifndef __DEBUG__H__
#define __DEBUG__H__

#include <iostream>
#include <experimental/filesystem>

#define DEBUG(message) \
    std::cerr << "# " << message << " -- " << \
    std::experimental::filesystem::path(__FILE__).filename().c_str() << \
    ":" << __LINE__ << std::endl

#define PLOT(message) \
    std::cout << "P " << message << std::endl

#endif
