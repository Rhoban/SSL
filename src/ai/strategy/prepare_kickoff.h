#ifndef __STRATEGY__PREPARE_KICKOFF__H__
#define __STRATEGY__PREPARE_KICKOFF__H__

#include "Strategy.h"
#include <string>
#include <robot_behavior/robot_behavior.h>
#include "placer.h"

namespace RhobanSSL {
namespace Strategy {


class Prepare_kickoff : public Strategy {
    private:
        bool is_kicking;
        bool strategy_is_active;
        Ai::RobotPlacement attacking_placement;
        Ai::RobotPlacement defending_placement;
        Placer placer_when_kicking;
        Placer placer_when_no_kicking;
        

   public:
        Prepare_kickoff(Ai::AiData & ai_data);
        virtual ~Prepare_kickoff();
        
        virtual int min_robots() const;
        virtual int max_robots() const;
        virtual Goalie_need needs_goalie() const;

        static const std::string name;

        virtual void start(double time);
        virtual void stop(double time);

        virtual void update(double time);
        void update_starting_positions();
        
        virtual void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );

        void set_positions(
            const std::vector<int> & robot_affectations,
            const std::vector<
                std::pair<rhoban_geometry::Point, ContinuousAngle>
            > & robot_consigns,
            bool allly_have_the_kickoff
        );
        void set_goalie_positions(
            const rhoban_geometry::Point & linear_position,
            const ContinuousAngle & angular_position,
            bool allly_have_the_kickoff
        );

        void set_kicking( bool value = true );

        virtual void set_robot_affectation( const std::vector<int> & robot_ids );

        virtual std::list<
            std::pair<rhoban_geometry::Point,ContinuousAngle>
        > get_starting_positions( int number_of_avalaible_robots ) ;  
        virtual bool get_starting_position_for_goalie(
            rhoban_geometry::Point & linear_position, 
            ContinuousAngle & angular_position
        ) ;  

}; 

};
};

#endif
