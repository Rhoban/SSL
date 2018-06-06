#ifndef __MANAGER__FACTORY__H__
#define __MANAGER__FACTORY__H__

#include <AiData.h>
#include <referee/Referee.h>
#include "Manager.h"

namespace RhobanSSL {
namespace Manager {

struct names {
    static constexpr const char* example = "example";
    static constexpr const char* example_for_testing_robot_behaviors = "example_for_testing_robot_behaviors";
    static constexpr const char* manual = "manual";
    static constexpr const char* match = "match";
};

class Factory {
    private:
    static std::list<std::string> list_of_avalaible_managers;
    
    public:
    static const std::list<std::string> & avalaible_managers();

    static std::shared_ptr<Manager> construct_manager(
        const std::string & manager_name,
        Ai::AiData & ai_data,
        Referee & referee
    );

};

};
};

#endif
