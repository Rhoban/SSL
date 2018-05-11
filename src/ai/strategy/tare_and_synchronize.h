#ifndef __STRATEGY__TARE_AND_SYNCHRONIZE__H__
#define __STRATEGY__TARE_AND_SYNCHRONIZE__H__

#include "Strategy.h"
#include <string>

namespace RhobanSSL {
namespace Strategy {


class Tare_and_synchronize : public Strategy {
    private:
        bool halt_behavior_was_assigned;
        bool move_behavior_was_assigned;
        bool time_is_synchro;
        double ai_time_command;
        
        double vision_time_command;
        double ai_time_associated_to_vision_time_command;

        void set_temporal_shift_between_vision();

    public:
        double get_temporal_shift_between_vision() const;

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
