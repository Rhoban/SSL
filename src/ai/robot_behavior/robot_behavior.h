#ifndef __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__
#define __ROBOT_BEHAVIOR__ROBOT_BEHAVIOR__H__

#include <game_informations.h>
#include <control/robot_control_with_position_following.h>
#include <control/robot_control_with_curve.h>
#include <rhoban_utils/angle.h>
#include <AiData.h>
#include <annotations/Annotations.h>

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

class RobotBehavior : public GameInformations {
    protected:
        const Ai::Robot* robot_ptr;
        const Ai::Ball* ball_ptr;
        double birthday;
        double lastUpdate;

        Vector2d robot_linear_position;
        ContinuousAngle robot_angular_position;
        Vector2d robot_linear_velocity;
        ContinuousAngle robot_angular_velocity;

        Ai::AiData & ai_data;
    public:
        RobotBehavior( Ai::AiData & ia_data );

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

        //
        // This function is used to draw annotations in the viewer.
        // You can use it to print what you want.
        //
        // For example :
        //
        // 
        //  RhobanSSLAnnotation::Annotations get_annotations() const{
        //      RhobanSSLAnnotation::Annotations annotations;
        //      static double d = 0;
        //      d += 0.01;
        //
        //      annotations.addCircle(3, 3, 1, "cyan");
        //      annotations.addArrow(0, 0, cos(d), sin(d)*2, "magenta", true);
        //      return annotations;
        //  }
        virtual RhobanSSLAnnotation::Annotations get_annotations() const;

        const Ai::Robot & robot() const ;
        const Ai::Ball & ball() const ;

        rhoban_geometry::Point linear_position() const ;
        ContinuousAngle angular_position() const;

        rhoban_geometry::Point ball_position() const ;
};

namespace detail {
    double vec2angle( Vector2d direction );
};

};
}; //Namespace Rhoban

#endif
