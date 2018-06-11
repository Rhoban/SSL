/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

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
