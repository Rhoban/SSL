#ifndef __STRATEGY__TARE_AND_SYNCHRONIZE__H__
#define __STRATEGY__TARE_AND_SYNCHRONIZE__H__

#include "Strategy.h"
#include <string>

namespace RhobanSSL {
namespace Strategy {


class Tare_and_synchronize : public Strategy {
    private:
        bool behavior_was_assigned;
        bool time_synchro;
        double waiting_time_for_synchro;
        double start_waiting_time_for_synchro;

    public:
        Tare_and_synchronize( Ai::AiData & ai_data );

        int min_robots() const;
        int max_robots() const;

        bool is_tared_and_synchronized() const;

        static const std::string name;

        void start(double time);
        void stop(double time);
        void update(double time);
        
        void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );
        virtual ~Tare_and_synchronize();
}; 

};
};

#endif
