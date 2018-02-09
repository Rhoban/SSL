#include "Referee.h"
#include <tools/debug.h>

namespace RhobanSSL {

void Referee::update( double time ){
    SSL_Referee referee_data = referee.getData();

    if( this->last_time == referee_data.packet_timestamp() ){
        return;
    }
    this->last_time = referee_data.packet_timestamp();

    DEBUG(
        "Commande " << referee_data.command()
    );
    if( referee_data.has_designated_position() ){
        const SSL_Referee_Point& position = referee_data.designated_position();
        DEBUG(
            "L'arbitre a dÃ©signce point : " << 
            position.x() << ", " << position.y() << 
            ")."
        );
    }
    
}

}
