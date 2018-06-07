#include "debug.h"

double last_debug_printing = 0;
bool periodic_debug_is_allowed = false;

void update_periodic_debug( double current_time, double period ){
    if(periodic_debug_is_allowed){
        periodic_debug_is_allowed= false;
    }else{
        if( last_debug_printing + period < current_time ){
            periodic_debug_is_allowed = true;
            last_debug_printing = current_time;
        }
    }
}
