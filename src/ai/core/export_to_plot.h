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
    void save( std::function< std::vector<double>() > fct );
    void save( const std::string & name_value, double value );
    void log();
    void close(); 
};

#endif
