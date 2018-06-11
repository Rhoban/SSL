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

#ifndef __LOGGER__H__
#define __LOGGER__H__

#include <core/export_to_plot.h>

#define LOGGER_LOG(logger_ptr, message) \
    { std::ostringstream s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger_ptr->log( s.str() ); \
    }

#define LOGGER_PRINT(logger_ptr, message) \
    { std::ostringstream s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger_ptr->print( s.str() ); \
    }

#define LOGGER_PRINT_AND_LOG(logger_ptr, message) \
    { std::ostringstream s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger_ptr->log( s.str() ); \
        logger_ptr->print( s.str() ); \
    }

struct Logger {
    
    bool active;
    Plot* plot;
    std::ofstream log_file;

    Logger();

    void init_plot(
        const std::string & name, const std::vector<std::string> & value_names 
    );
    void init_file( const std::string & file_name );

    void log_plot( const std::string & name, double value );
    void log_plot( std::function< std::vector<double>() > fct );
    void store_plot();

    Plot* get_plot();

    bool is_active() const;
    void log( const std::string & message );

    void print( const std::string & message ) const;

    void activate();
    void desactivate();


    virtual ~Logger();
};

#endif
