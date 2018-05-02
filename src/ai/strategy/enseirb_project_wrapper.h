#ifndef __STRATEGY__ENSEIRB_PROJECT_WRAPPER__H__
#define __STRATEGY__ENSEIRB_PROJECT_WRAPPER__H__

#include "Strategy.h"
#include <string>
#include <robot_behavior/robot_behavior.h>
#include "enseirb_projects/api.h"

namespace RhobanSSL {
namespace Strategy {


class Enseirb_project_wrapper : public Strategy {
    private:
        void allocate_enseirb_data();
        void desallocate_enseirb_data();
        void initialize_enseirb_data();

    private:
        Ai::AiData & game_state;
        bool behavior_has_been_assigned;

        // std::vector<int, std::pair<Vision::Team, int> > index_2_robot;
        std::map<std::pair<Vision::Team, int>, int> robot_2_index;
        std::vector<enseirb::Action> robot_actions;
        enseirb::Ball ball;
        struct enseirb::Config config;
        struct enseirb::Robot * robots;
        int nb_robots;

    public:
        Enseirb_project_wrapper(Ai::AiData & game_state);

        virtual int min_robots() const;
        virtual int max_robots() const;

        static const std::string name;

        virtual void start(double time);
        virtual void stop(double time);

        virtual void update(double time);

        virtual void assign_behavior_to_robots(
            std::function<
                void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
            > assign_behavior,
            double time, double dt
        );
        virtual ~Enseirb_project_wrapper();
}; 

};
};

#endif
