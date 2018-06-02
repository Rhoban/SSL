#ifndef __STRATEGY__FROM_ROBOT_BEHAVIOR__H__
#define __STRATEGY__FROM_ROBOT_BEHAVIOR__H__

#include "Strategy.h"
#include <string>
#include <memory>

namespace RhobanSSL {
namespace Strategy {


class From_robot_behavior: public Strategy {
    private:
        std::function<
            std::shared_ptr<Robot_behavior::RobotBehavior> (double time, double dt) 
        > robot_behavior_allocator;
        bool behavior_has_been_assigned;
        bool is_goalie;
    
        struct StartingPosition {
            bool is_defined;
            rhoban_geometry::Point linear_position; 
            ContinuousAngle angular_position;
            StartingPosition() : is_defined(false) {};
        };
        StartingPosition starting_position;

        

    public:

        From_robot_behavior( 
            Ai::AiData & ai_data,
            std::function<
                std::shared_ptr<Robot_behavior::RobotBehavior>(double time, double dt)
            > robot_behavior_allocator,
            bool is_goalie = false
        );
        virtual int min_robots() const;
        virtual int max_robots() const;

        void set_starting_position(
            const rhoban_geometry::Point & linear_position, 
            const ContinuousAngle & angular_position
        );  

        static const std::string name;

        virtual void start(double time);
        virtual void stop(double time);

        virtual Goalie_need needs_goalie() const;
        
        virtual void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );


        virtual std::list<
            std::pair<rhoban_geometry::Point,ContinuousAngle>
        > get_starting_positions( int number_of_avalaible_robots ) const;  
        virtual bool get_starting_position_for_goalie(
            rhoban_geometry::Point & linear_position, 
            ContinuousAngle & angular_position
        ) const;  


        virtual ~From_robot_behavior();
}; 


};
};

#endif
