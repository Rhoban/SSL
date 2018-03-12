#ifndef __CORE__EXPORT_TO_PLOT__H__
#define __CORE__EXPORT_TO_PLOT__H__
#include <fstream>
#include <functional>
#include <vector>

class Plot {
    std::ofstream log_file;
    
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
    void close(); 
};

#endif
