#include "Movement.h"

std::ostream& operator<<(
    std::ostream& stream, const RhobanSSL::Movement& movement
){
    movement.print( stream );
    return stream;
}

