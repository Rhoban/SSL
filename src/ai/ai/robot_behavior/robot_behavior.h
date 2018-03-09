#ifndef __ROBOT_BEHAVIOR__H__
#define __ROBOT_BEHAVIOR__H__

#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <geometry/Angle.hpp>
#include <ai/AiData.h>

namespace RhobanSSL {

struct Control : PidControl {
    bool kick;
    bool active;
    bool ignore;

    Control();
    Control(bool kick, bool active, bool ignore);
    Control(const PidControl& c);

    static Control make_desactived();
    static Control make_ignored();
    static Control make_null();
};

std::ostream& operator << ( std::ostream &, const Control& control  );

class RobotBehavior {
    protected:
    
        double birthday;
        double lastUpdate; 

        Eigen::Vector2d robot_linear_position;
        ContinuousAngle robot_angular_position;
        Eigen::Vector2d ball_position;

    public:
        RobotBehavior();
        
        double age() const;
        bool is_born() const;
        double set_birthday( double birthday );

        void update_time_and_position(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual void update(
            double time, 
            const Ai::Robot & robot, const Ai::Ball & ball
        ) = 0;
        virtual Control control() const = 0;
};

namespace detail {
    double vec2angle( Eigen::Vector2d direction );
};

}; //Namespace Rhoban

#endif
