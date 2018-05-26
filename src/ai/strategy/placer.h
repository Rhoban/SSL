#ifndef __STRATEGY__PLACER__H__
#define __STRATEGY__PLACER__H__

#include "Strategy.h"
#include <string>
#include <list>
#include <robot_behavior/robot_behavior.h>

namespace RhobanSSL {
namespace Strategy {


class Placer : public Strategy {
    private:
        std::map<
            int,
            std::pair<rhoban_geometry::Point, ContinuousAngle>
        > player_positions;
        std::vector< 
            std::pair<rhoban_geometry::Point, ContinuousAngle> 
        > wished_robot_positions;
    public:

        Placer(Ai::AiData & ai_data);
        bool behavior_has_been_assigned;
        int min_robots() const;
        int max_robots() const;

        // Try to place one robot at each given position.
        // This function return which robot id have been placed. 
        // the order of robot id correspond to the order of the given robot position
        void set_positions(
            const std::vector<int> & robot_affectations,
            const std::vector<
                std::pair<rhoban_geometry::Point, ContinuousAngle>
            > & robot_consigns
        );
        void set_goalie_positions(
            const rhoban_geometry::Point & linear_position,
            const ContinuousAngle & angular_position
        );
        void ignore_goalie();

        static const std::string name;

        void set_starting_position(
            const std::vector< rhoban_geometry::Point > & starting_position
        );

        void start(double time);
        void stop(double time);

        void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );
        virtual ~Placer();
}; 

};
};

#endif
