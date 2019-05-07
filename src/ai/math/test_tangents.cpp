/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
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
#include "tangents.h"
#include "vector2d.h"

using namespace rhoban_geometry;

TEST(test_tangents, center_of_cone_incircle)
{
  {
    rhoban_geometry::Point cone_vertex(0.0, 0.0);
    rhoban_geometry::Point cone_base_A(0.0, -2.0);
    rhoban_geometry::Point cone_base_B(-2.0, 0.0);
    double circle_radius = 1.0;
    rhoban_geometry::Point circle_center =

        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(-1.0, -1.0)) < 0.001);
  }
  {
    rhoban_geometry::Point cone_vertex(0.0, 0.0);
    rhoban_geometry::Point cone_base_A(-2.0, 0.0);
    rhoban_geometry::Point cone_base_B(0.0, -2.0);
    double circle_radius = 1.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(-1.0, -1.0)) < 0.001);
  }
  {
    rhoban_geometry::Point cone_vertex(2.0, 3.0);
    rhoban_geometry::Point cone_base_A(0.0, 3.0);
    rhoban_geometry::Point cone_base_B(2.0, 1.0);
    double circle_radius = 1.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(1.0, 2.0)) < 0.001);
  }
  {
    rhoban_geometry::Point cone_vertex(2.0, 3.0);
    rhoban_geometry::Point cone_base_A(2.0, 1.0);
    rhoban_geometry::Point cone_base_B(0.0, 3.0);
    double circle_radius = 1.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(1.0, 2.0)) < 0.001);
  }
  {
    double d = std::sqrt(3) / 2.0;
    rhoban_geometry::Point cone_vertex(1.0 / 2.0, d);
    rhoban_geometry::Point cone_base_A(0.0, 0.0);
    rhoban_geometry::Point cone_base_B(1.0, 0.0);
    double circle_radius = d / 3.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(1.0 / 2.0, d / 3.0)) < 0.00001);
  }
  {
    double d = std::sqrt(3) / 2.0;
    rhoban_geometry::Point cone_vertex(1.0 / 2.0, d);
    rhoban_geometry::Point cone_base_B(0.0, 0.0);
    rhoban_geometry::Point cone_base_A(1.0, 0.0);
    double circle_radius = d / 3.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(1.0 / 2.0, d / 3.0)) < 0.00001);
  }
  {
    double d = std::sqrt(3) / 2.0;
    rhoban_geometry::Point cone_vertex(2 + 1.0 / 2.0, 3 + d);
    rhoban_geometry::Point cone_base_A(2 + 0.0, 3 + 0.0);
    rhoban_geometry::Point cone_base_B(2 + 1.0, 3 + 0.0);
    double circle_radius = d / 3.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(2 + 1.0 / 2.0, 3 + d / 3.0)) < 0.00001);
  }
  {
    double d = std::sqrt(3) / 2.0;
    rhoban_geometry::Point cone_vertex(2 + 1.0 / 2.0, 3 + d);
    rhoban_geometry::Point cone_base_B(2 + 0.0, 3 + 0.0);
    rhoban_geometry::Point cone_base_A(2 + 1.0, 3 + 0.0);
    double circle_radius = d / 3.0;
    rhoban_geometry::Point circle_center =
        centerOfConeIncircle(cone_vertex, cone_base_A, cone_base_B, circle_radius);
    EXPECT_TRUE(norm(circle_center - rhoban_geometry::Point(2 + 1.0 / 2.0, 3 + d / 3.0)) < 0.00001);
  }
}

TEST(test_tangents, tangent_of_two_circle)
{
  {
    rhoban_geometry::Circle circle_A(rhoban_geometry::Point(0.0, 0.0), 1.0);
    rhoban_geometry::Circle circle_B(rhoban_geometry::Point(3.0, 0.0), 1.0);
    std::vector<rhoban_geometry::Segment> tangents = tangentOfTwoCircle(circle_A, circle_B);
    EXPECT_EQ(tangents[0].A, rhoban_geometry::Point(0.0, 1.0));
    EXPECT_EQ(tangents[0].B, rhoban_geometry::Point(3.0, 1.0));
    EXPECT_EQ(tangents[1].A, rhoban_geometry::Point(0.0, -1.0));
    EXPECT_EQ(tangents[1].B, rhoban_geometry::Point(3.0, -1.0));

    double d = 3.0;
    double a = std::sqrt((d / 2) * (d / 2) - 1);
    double y = 1.0 * a / (d / 2.0);
    double x = std::sqrt(1 - y * y);
    EXPECT_TRUE(norm(tangents[2].A - rhoban_geometry::Point(x, y)) < 0.0001);
    EXPECT_TRUE(norm(tangents[2].B - rhoban_geometry::Point(3.0 - x, -y)) < 0.0001);
    EXPECT_TRUE(norm(tangents[3].A - rhoban_geometry::Point(x, -y)) < 0.0001);
    EXPECT_TRUE(norm(tangents[3].B - rhoban_geometry::Point(3.0 - x, y)) < 0.0001);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
