#ifndef __STRATEGY__STRATEGY__H__
#define __STRATEGY__STRATEGY__H__

#include <robot_behavior/robot_behavior.h>
#include <map>
#include <utility>
#include <math/ContinuousAngle.h>
#include <memory>
#include <AiData.h>

namespace RhobanSSL {
namespace Strategy {

enum Goalie_need {
    YES,
    IF_POSSIBLE,
    NO
};

class Strategy {
    protected:
    Ai::AiData & ai_data;
    private:
    int goalie_id;
    bool manage_a_goalie;

    int goalie_opponent_id;
    std::vector<int> player_ids;

    void update_player_ids();

    public:

    Strategy(Ai::AiData & ai_data);

    void set_goalie( int id, bool to_be_managed );
    void set_goalie_opponent( int id );
    
    // Get the goalie id. If id<0 then no goalie is declared 
    int get_goalie() const;   
    // Get the opponent goalie id. If id<0 then no opponent goalie is declared  
    int get_goalie_opponent() const;   

    /* 
     * This function is called by the manager to affect robot to the stratgey.
     * Here only robot for field player are affected.
     * The robot id for the goal can only be obtained from the function get_goalie().
     */
    void set_robot_affectation( const std::vector<int> & robot_ids );
    
    const std::vector<int> & get_player_ids() const;

    int robot_id( int id ) const;
    int player_id( int id ) const;


    virtual void update(double time){};

    virtual void start(double time){};
    virtual void stop(double time){};
    virtual void pause(double time){};
    virtual void resume(double time){};

    /* 
     * Return a list of position where it is recommended to place a 
     * robot before starting the strategy.
     * the size of the list should be smaller than the parameter 
     * number_of_avalaible_robots.
     */
    virtual std::list<
        std::pair<rhoban_geometry::Point,ContinuousAngle>
    > get_starting_positions( int number_of_avalaible_robots ) const;  
    /* 
     * Set the position where it is recommended to place a goalie 
     * before starting the strategy.
     * If this function return false, then no position is given for a goale.
     * If the strategy have no goalie, this function have to return false.
     */
    virtual bool get_starting_position_for_goalie(
        rhoban_geometry::Point & linear_position, 
        ContinuousAngle & angular_position
    ) const;  

    /*
     * This function give the minimal numer of non goalie robot 
     * that the strategy commands.
     */
    virtual int min_robots() const = 0;
    /*
     * This function give the maximal numer of non goalie robot 
     * that strategy commands.
     * if it is set to -1, then the strategy can command any number of robot.
     */
    virtual int max_robots() const = 0;

    /*
     * Say if the strategy need a goalie.
     */
    virtual Goalie_need needs_goalie() const = 0;

    virtual void assign_behavior_to_robots(
        std::function<
            void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
        > assign_behavior,
        double time, double dt
    ) = 0;

    virtual ~Strategy();

    bool have_to_manage_the_goalie() const;

};

};
};
#endif
