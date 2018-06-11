/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/
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
