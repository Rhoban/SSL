#ifndef __CORE__ASSERTION__H__
#define __CORE__ASSERTION__H__

#ifndef NDEBUG
    #define assert_message(cond, message) {std::cerr << message << std::endl; assert(cond)}
#else
    #define assert_message(cond, message) assert(cond)
#enduf

#endif
