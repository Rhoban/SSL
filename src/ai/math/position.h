#ifndef __MATH__POSITION__H__
#define __MATH__POSITION__H__

#include "ContinuousAngle.h"
#include <rhoban_geometry/point.h>
#include <iostream>

struct Position {
    rhoban_geometry::Point linear;
    ContinuousAngle angular;

    Position();
    Position(
        const rhoban_geometry::Point & linear,
        const ContinuousAngle & angular
    );
    Position(double x, double y, double angle);
};

std::ostream& operator<<(std::ostream& out, const Position& pos);

#endif
