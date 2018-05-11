#ifndef __MOVEMENT_WITH_TEMPORAL_SHIFT__H__
#define __MOVEMENT_WITH_TEMPORAL_SHIFT__H__

#include <physic/Movement.h>

namespace RhobanSSL {

class Movement_with_temporal_shift : public Movement {
    private:
        Movement* movement;
        std::function< double () > temporal_shift;

    public:

        //We assume that v1 and v2 are orthonormal
        void set_shift( double shift_time );

        virtual double last_time() const;

        virtual Movement * clone() const;
        const Movement* get_original_movement() const;

        Movement_with_temporal_shift(
            Movement* movement,
            std::function< double () > temporal_shift
        );

        virtual void set_sample( const MovementSample & samples );
        virtual const MovementSample & get_sample() const;

        virtual rhoban_geometry::Point linear_position( double time ) const;
        virtual ContinuousAngle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual ContinuousAngle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual ContinuousAngle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;

        virtual ~Movement_with_temporal_shift();
};


}

#endif
