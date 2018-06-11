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

#ifndef __CORE__EXPORT_TO_PLOT__H__
#define __CORE__EXPORT_TO_PLOT__H__
#include <fstream>
#include <functional>
#include <vector>
#include <map>

class Plot {
    std::ofstream log_file;
    
    std::vector<std::string> value_names;
//    std::map<std::string, int> value_name_id;
    std::map<std::string, double> current_values;
    std::map<std::string, bool> loged_values;
    int n;

    void create_plot_script(
        const std::vector<std::string> & value_names
    );

    public: 
    std::string name;

    virtual ~Plot();
        
    void init( 
        const std::string & name, const std::vector<std::string> & value_names 
    );
    void log( std::function< std::vector<double>() > fct );
    void log( const std::string & name_value, double value );
    void store();
    void close(); 
};

#endif
