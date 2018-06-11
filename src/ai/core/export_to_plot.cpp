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

#include "export_to_plot.h"
#include <iostream>
#include <debug.h>
#include <iomanip>
#include <limits>

void Plot::create_plot_script(
    const std::vector<std::string> & value_names
){
    std::string file_name = (name + ".plot");
    std::ofstream file( file_name );
    if( ! file.is_open() ){
        std::cerr << "ERROR : It is not possible to write in " << file_name << std::endl;
        return ;
    }
    file << "plot" ;
    for( int i=1; i < n; i++ ){
        file << " \"" << name + ".log\"";
        file <<  " u 1:" << i+1  << " t \"" << value_names[i] << "\"";
        file << " w lp,";
    }
    file << std::endl;
    file.close();
}

void Plot::init(
    const std::string & name, 
    const std::vector<std::string> & value_names
){
    this->name = name;
    this->value_names = value_names;
    this->n = value_names.size();
    for( int i=0; i<n; i++ ){
        current_values[ value_names[i] ] = 0.0;
        loged_values[ value_names[i] ] = false;
    }
    Plot::create_plot_script(value_names);
    log_file.open(name+".log");
    if( !log_file.is_open()){
        std::cerr << "ERROR : It is not possible to write in " << name << ".log" << std::endl;
    }
    log_file << std::setprecision(std::numeric_limits<double>::digits10 + 1); 
}

void Plot::close(){
    if( log_file.is_open() ){
        log_file.close();
    }
}

void Plot::store(){
    if( ! log_file.is_open() ) return;
    if( loged_values[value_names[0]] ){
        for( int i=0; i<n; i++ ){
            if( ! loged_values[value_names[i]] ){
                std::cerr <<
                    "Value missing for " << value_names[i] << " at " << current_values[value_names[0]] << std::endl;
            }
            log_file << current_values[ value_names[i] ] << " ";
        }
        log_file << std::endl;
    }
    for( int i=0; i<n; i++ ){
        current_values[ value_names[i] ] = 0.0;
        loged_values[ value_names[i] ] = false;
    }
}

void Plot::log( std::function< std::vector<double>() > fct ){
    for( int i=0; i<n; i++ ){
        current_values[ value_names[i] ] = fct()[i];
        loged_values[ value_names[i] ] = true;
    }
}

void Plot::log( const std::string & name_value, double value ){
    current_values.at( name_value ) = value;
    loged_values.at( name_value ) = true;
}

Plot::~Plot(){
    close();
}
