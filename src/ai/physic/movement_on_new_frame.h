#ifndef __MOVEMENT_ON_NEW_FRAME_H__ 
#define __MOVEMENT_ON_NEW_FRAME_H__ 

#include <physic/Movement.h>
#include <math/frame_changement.h>

namespace RhobanSSL {

class Movement_on_new_frame : public Movement {
    private:
        Movement* movement;
        Frame_changement frame;

    public:

        //We assume that v1 and v2 are orthonormal
        void set_frame(
            const rhoban_geometry::Point & origin,
            const Vector2d & v1, const Vector2d & v2
        );

        virtual Movement * clone() const;
        const Movement* get_original_movement() const;

        Movement_on_new_frame(Movement* movement);

        virtual void set_sample( const MovementSample & samples );
        virtual const MovementSample & get_sample() const;

        virtual rhoban_geometry::Point linear_position( double time ) const;
        virtual ContinuousAngle angular_position( double time ) const;

        virtual Vector2d linear_velocity( double time ) const;
        virtual ContinuousAngle angular_velocity( double time ) const;

        virtual Vector2d linear_acceleration( double time ) const;
        virtual ContinuousAngle angular_acceleration( double time ) const;

        virtual void print(std::ostream& stream) const;

        virtual ~Movement_on_new_frame();
};


}

#endif
