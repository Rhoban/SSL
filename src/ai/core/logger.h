#ifndef __LOGGER__H__
#define __LOGGER__H__

#include <core/export_to_plot.h>

#define LOG(logger, message) \
    { std::ostringstream & s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger.log( s.str() ); \
    }

#define PRINT(logger, message) \
    { std::ostringstream & s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger.print( s.str() ); \
    }

#define PRINT_AND_LOG(logger, message) \
    { std::ostringstream & s;\
        s << "# " << message << " -- " << \
        rhoban_utils::getBaseName(__FILE__) << \
        ":" << __LINE__ << std::endl; \
        logger.log( s.str() ); \
        logger.print( s.str() ); \
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
