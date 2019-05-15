/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com) (Refacto)

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

#include <gtest/gtest.h>

#include <debug.h>
#include "lines.h"
#include "vector2d.h"

TEST(test_lines, distance_from_point_to_line)
{
  {
    rhoban_geometry::Point point(0.5, 0.0);
    rhoban_geometry::Point point_line_1(0.0, 0.0);
    rhoban_geometry::Point point_line_2(1.0, 0.0);

    double distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(distance < 0.0000000001);

    point = rhoban_geometry::Point(0.0, 1.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(((distance < 1.0000000001) && (distance > 0.9999999999)));

    point = rhoban_geometry::Point(-2.0, -2.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(((distance < 2.0000000001) && (distance > 1.9999999999)));
  }
  {
    rhoban_geometry::Point point(0.5, 0.0);
    rhoban_geometry::Point point_line_1(0.0, 0.0);
    rhoban_geometry::Point point_line_2(0.0, 1.0);

    double distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(((distance < 0.5000000001) && (distance > 0.4999999999)));

    point = rhoban_geometry::Point(0.0, 1.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(distance < 0.0000000001);

    point = rhoban_geometry::Point(-2.0, -2.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE(((distance < 2.0000000001) && (distance > 1.9999999999)));
  }
  {
    rhoban_geometry::Point point(1.0, 0.0);
    rhoban_geometry::Point point_line_1(0.0, 0.0);
    rhoban_geometry::Point point_line_2(1.0, 1.0);

    double distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);

    EXPECT_TRUE(((distance < 0.7071080001) && (distance > 0.7070999999)));

    point = rhoban_geometry::Point(0.0, 1.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);

    EXPECT_TRUE(((distance < 0.7071080001) && (distance > 0.7070999999)));

    point = rhoban_geometry::Point(-2.0, -2.0);
    distance = rhoban_geometry::distanceFromPointToLine(point, point_line_1, point_line_2);
    EXPECT_TRUE((distance < 0.0000000001));
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
