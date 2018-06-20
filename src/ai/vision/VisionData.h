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

#ifndef __VISIONDATA_H__
#define __VISIONDATA_H__

#include <map>
#include <rhoban_geometry/point.h>
#include <math/ContinuousAngle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/MovementSample.h>
#include <iostream>

namespace RhobanSSL {
namespace Vision {

static const int history_size = 129;
static const int Robots = 16;

typedef enum {
    Ally,
    Opponent
} Team;


struct Object {
    MovementSample movement;

    bool present;
    int id;
    rhoban_utils::TimeStamp lastUpdate;

    void update(
        double time, const rhoban_geometry::Point & linear_position,
        const rhoban_utils::Angle & angular_position
    );
    void update(
        double time, const rhoban_geometry::Point & linear_position,
        const ContinuousAngle & angular_position
    );
    void update(
        double time, const rhoban_geometry::Point & linear_position
    );

    double age() const;
    bool isOk() const;
    bool is_too_old() const;

    Object();
    void checkAssert( double time ) const;
};

std::ostream& operator<<(std::ostream& out, const Object& object);

struct Robot : Object { };
struct Ball : Object { };

struct Field
{
    bool present;
    float fieldLength;
    float fieldWidth;
    float goalWidth;
    float goalDepth;
    float boundaryWidth;
    float penaltyAreaDepth;
    float penaltyAreaWidth;

    Field();
};
std::ostream& operator<<(std::ostream& out, const Field& field);



class VisionData {
public:
    VisionData();

    std::map<Team, std::map<int, Robot>> robots;
    Ball ball;
    Field field;

    double older_time() const;
    void checkAssert( double time ) const;

    void print() const;

    friend std::ostream& operator<<(std::ostream& out, const RhobanSSL::Vision::VisionData& vision);

};

std::ostream& operator<<(std::ostream& out, const VisionData& vision);

} }


#endif
