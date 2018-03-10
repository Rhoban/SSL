#ifndef __MOVEMENT_WITH_NO_PREDICTION_H__ 
#define __MOVEMENT_WITH_NO_PREDICTION_H__ 

#include <physic/Movement.h>

namespace RhobanSSL {

class Movement_with_no_prediction : public Movement {
    private:
        MovementSample samples;

    public:
        virtual Movement * clone() const;

        virtual void set_sample( const MovementSample & samples );
        virtual const MovementSample & get_sample() const;

        virtual Vector2d linear_position( double time ) const;
        virtual ContinuousAngle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual ContinuousAngle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual ContinuousAngle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;
};


}

#endif
