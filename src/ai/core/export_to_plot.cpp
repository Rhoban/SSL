#include "export_to_plot.h"
#include <iostream>

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
    this->n = value_names.size();
    Plot::create_plot_script(value_names);
    log_file = std::ofstream(name+".log");
    if( !log_file.is_open()){
        std::cerr << "ERROR : It is not possible to write in " << name << ".log" << std::endl;
    }
}

void Plot::close(){
    if( log_file.is_open() ){
        log_file.close();
    }
}

void Plot::log( std::function< std::vector<double>() > fct ){
    if( ! log_file.is_open() ) return;
    for( int i=0; i<n; i++ ){
        log_file << fct()[i] << " ";
    }
    log_file << std::endl;
}

Plot::~Plot(){
    close();
}
