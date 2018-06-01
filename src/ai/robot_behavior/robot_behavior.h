#ifndef __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__
#define __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__

#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <rhoban_utils/angle.h>
#include <AiData.h>

namespace RhobanSSL {

struct Control : PidControl {
    bool kick;
    bool chipKick;
    int kickPower;
    bool spin;
    bool charge;
    bool active;
    bool ignore;

    Control();
    Control(bool kick, bool active, bool ignore);
    Control(const PidControl& c);

    static Control make_desactivated();
    static Control make_ignored();
    static Control make_null();
};

std::ostream& operator << ( std::ostream &, const Control& control  );

namespace Robot_behavior {

class RobotBehavior {
    protected:
        const Ai::Robot* robot_ptr;
        const Ai::Ball* ball_ptr;
        double birthday;
        double lastUpdate;

        Vector2d robot_linear_position;
        ContinuousAngle robot_angular_position;
        Vector2d robot_linear_velocity;
        ContinuousAngle robot_angular_velocity;
        Vector2d ball_position;

        Ai::AiData & ai_data;
    public:
        RobotBehavior( Ai::AiData & ia_data );

        double time() const;
        double age() const;
        bool is_born() const;
        void set_birthday( double birthday );

        void update_time_and_position(
            double time,
            const Ai::Robot & robot, const Ai::Ball & ball
        );

        virtual void update(
            double time,
            const Ai::Robot & robot, const Ai::Ball & ball
        ) = 0;
        virtual Control control() const = 0;

        const Ai::Robot & robot() const ;
        const Ai::Ball & ball() const ;
        const rhoban_geometry::Point ally_goal_center() const ;
        const rhoban_geometry::Point oponent_goal_center() const ;
        const rhoban_geometry::Point center_mark() const ;
};

namespace detail {
    double vec2angle( Vector2d direction );
};

};
}; //Namespace Rhoban

#endif
