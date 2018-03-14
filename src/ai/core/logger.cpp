#include "logger.h"
#include <iostream>
#include <sstream>
#include <debug.h>

Logger::Logger(): active(false), plot(new Plot()) { } 

void Logger::init_plot(
    const std::string & name, const std::vector<std::string> & value_names 
){
    plot->init( name, value_names );
}
void Logger::init_file( const std::string & file_name ){
    log_file.open( file_name );
    if( ! log_file.is_open() ){
        std::cerr << "ERROR : It is not possible to write in " << file_name << std::endl;
        return ;
    }
}

void Logger::log_plot( const std::string & name, double value ){
    if( active ){
        if( plot ){
            plot->log( name, value);
        }
    }
}

void Logger::log_plot( std::function< std::vector<double>() > fct ){
    if( active ){
        if( plot ){
            plot->log( fct );
        }
    }
}

void Logger::store_plot(){
    if( plot ){
        plot->store();
    }
}

bool Logger::is_active() const {
    return active;
}

void Logger::log( const std::string & message ){
    if( is_active() ){
        if( log_file.is_open() ){
            log_file << message;
        }
    }
}

void Logger::print( const std::string & message ) const {
    if( is_active() ){
        std::cout << message;
    }
}

void Logger::activate(){
    active = true;
}

void Logger::desactivate(){
    active = false;
}

Plot* Logger::get_plot(){
    return plot;
}

Logger::~Logger(){
    delete plot;
}
