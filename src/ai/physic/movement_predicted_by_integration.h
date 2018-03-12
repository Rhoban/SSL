#ifndef __MOVEMENT_PREDICTION_BY_INTEGRATION_H__ 
#define __MOVEMENT_PREDICTION_BY_INTEGRATION_H__ 

#include <physic/Movement.h>

namespace RhobanSSL {

class Movement_predicted_by_integration : public Movement {
    private:
        MovementSample samples;

        void check();

    public:
        virtual Movement * clone() const;

        virtual void set_sample( const MovementSample & samples );
        virtual const MovementSample & get_sample() const;

        virtual rhoban_geometry::Point linear_position( double time ) const;
        virtual ContinuousAngle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual ContinuousAngle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual ContinuousAngle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;
};

}

#endif
