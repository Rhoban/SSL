/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)
    Copyright 2019 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)


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

#ifndef __CONTROL__H__
#define __CONTROL__H__

#include <math/ContinuousAngle.h>
#include <math/vector2d.h>

class Control
{
private:
    bool m_is_absolute;
public :
    Vector2d linear_velocity = Vector2d(0,0);
    ContinuousAngle angular_velocity = ContinuousAngle(0);

    bool charge   = false;
    bool kick     = false;
    bool chipKick = false;
    float kickPower = 1.0;
    bool spin     = false;

    bool active   = true;
    bool ignore   = false;

    Control(bool is_absolute = true);

    Control(const Vector2d & linear_velocity,
        const ContinuousAngle &angular_velocity,
        bool is_absolute = true
    );

    Control(bool kick, bool active, bool ignore);

    void change_to_relative_control(
        const ContinuousAngle & robot_orientation,
        double dt );

    void change_to_absolute_control(
        const ContinuousAngle & robot_orientation,
        double dt );

    bool is_absolute();

    bool is_relative();

    static Control make_desactivated();
    static Control make_ignored();
    static Control make_null();
};

std::ostream& operator << ( std::ostream & out, const Control& control );

#endif