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
#include "movement_sample.h"
#include <math.h>

using namespace rhoban_geometry;

TEST(test_MovementSample, position_sample)
{
  {
    rhoban_ssl::PositionSample pos;
    EXPECT_TRUE(pos.time == 0);
    EXPECT_TRUE(pos.linear_position == Point(0.0, 0.0));
    EXPECT_TRUE(pos.angular_position == ContinuousAngle(0.0));
  }

  {
    rhoban_ssl::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));
    EXPECT_TRUE(pos.time == 3.0);
    EXPECT_TRUE(pos.linear_position == Point(4.0, 5.0));
    EXPECT_TRUE(pos.angular_position == ContinuousAngle(6.0));
  }

  {
    rhoban_ssl::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));

    std::ostringstream s1;
    s1 << pos;

    EXPECT_TRUE("(t=3, lin={4,5}, ang=6)" == s1.str());
  }
}

TEST(test_MovementSample, some_use_cases)
{
  {
    rhoban_ssl::MovementSample mov;
    EXPECT_TRUE(mov.size() == 0);
  }
  {
    rhoban_ssl::MovementSample mov(3);
    mov.insert(rhoban_ssl::PositionSample(1, Point(1, 2), ContinuousAngle(4)));
    mov.insert(rhoban_ssl::PositionSample(2, Point(8, 16), ContinuousAngle(32)));
    mov.insert(rhoban_ssl::PositionSample(3, Point(27, 54), ContinuousAngle(108)));

    EXPECT_TRUE(mov.linearPosition(2) == Point(1, 2));
    EXPECT_TRUE(mov.linearPosition(1) == Point(8, 16));
    EXPECT_TRUE(mov.linearPosition(0) == Point(27, 54));

    EXPECT_TRUE(mov.angularPosition(2) == ContinuousAngle(4));
    EXPECT_TRUE(mov.angularPosition(1) == ContinuousAngle(32));
    EXPECT_TRUE(mov.angularPosition(0) == ContinuousAngle(108));

    EXPECT_TRUE(mov.linearVelocity(1) == Point(7, 14));
    EXPECT_TRUE(mov.linearVelocity(0) == Point(19, 38));

    EXPECT_TRUE(mov.angularVelocity(1) == ContinuousAngle(28));
    EXPECT_TRUE(mov.angularVelocity(0) == ContinuousAngle(76));

    EXPECT_TRUE(mov.linearAcceleration(0) == Vector2d(12, 24));

    EXPECT_TRUE(mov.angularAcceleration(0) == ContinuousAngle(48));
  }
  {
    rhoban_ssl::MovementSample mov(3);
    mov.insert(rhoban_ssl::PositionSample(1, Point(1, 2), ContinuousAngle(4)));
    mov.insert(rhoban_ssl::PositionSample(3, Point(8, 16), ContinuousAngle(32)));
    mov.insert(rhoban_ssl::PositionSample(6, Point(27, 54), ContinuousAngle(108)));

    EXPECT_TRUE(mov.linearPosition(2) == Point(1, 2));
    EXPECT_TRUE(mov.linearPosition(1) == Point(8, 16));
    EXPECT_TRUE(mov.linearPosition(0) == Point(27, 54));

    EXPECT_TRUE(mov.angularPosition(2) == ContinuousAngle(4));
    EXPECT_TRUE(mov.angularPosition(1) == ContinuousAngle(32));
    EXPECT_TRUE(mov.angularPosition(0) == ContinuousAngle(108));

    EXPECT_TRUE(mov.linearVelocity(1) == Point(7, 14) / 2);
    EXPECT_TRUE(mov.linearVelocity(0) == Point(19, 38) / 3);

    EXPECT_TRUE(mov.angularVelocity(1) == ContinuousAngle(28) / 2);
    EXPECT_TRUE(mov.angularVelocity(0) == ContinuousAngle(76) / 3);

    EXPECT_TRUE(norm(mov.linearAcceleration(0) - Vector2d(17, 34) / (2 * 3 * 3)) < 0.0001);

    EXPECT_TRUE(std::fabs((mov.angularAcceleration(0) - ContinuousAngle(68) / (2 * 3 * 3)).value()) < 0.0001);
  }
  {
    rhoban_ssl::MovementSample mov(4);
    mov.insert(rhoban_ssl::PositionSample(1, Point(1, 2), ContinuousAngle(4)));
    mov.insert(rhoban_ssl::PositionSample(3, Point(8, 16), ContinuousAngle(32)));
    mov.insert(rhoban_ssl::PositionSample(6, Point(27, 54), ContinuousAngle(108)));
    mov.insert(rhoban_ssl::PositionSample(10, Point(343, 686), ContinuousAngle(1029)));

    EXPECT_TRUE(mov.linearPosition(3) == Point(1, 2));
    EXPECT_TRUE(mov.linearPosition(2) == Point(8, 16));
    EXPECT_TRUE(mov.linearPosition(1) == Point(27, 54));
    EXPECT_TRUE(mov.linearPosition(0) == Point(343, 686));

    EXPECT_TRUE(mov.angularPosition(3) == ContinuousAngle(4));
    EXPECT_TRUE(mov.angularPosition(2) == ContinuousAngle(32));
    EXPECT_TRUE(mov.angularPosition(1) == ContinuousAngle(108));
    EXPECT_TRUE(mov.angularPosition(0) == ContinuousAngle(1029));

    EXPECT_TRUE(mov.linearVelocity(2) == Point(7, 14) / 2);
    EXPECT_TRUE(mov.linearVelocity(1) == Point(19, 38) / 3);
    EXPECT_TRUE(mov.linearVelocity(0) == Point(316, 632) / 4);

    EXPECT_TRUE(mov.angularVelocity(2) == ContinuousAngle(28) / 2);
    EXPECT_TRUE(mov.angularVelocity(1) == ContinuousAngle(76) / 3);
    EXPECT_TRUE(mov.angularVelocity(0) == ContinuousAngle(921) / 4);

    EXPECT_TRUE(norm(mov.linearAcceleration(1) - Vector2d(17, 34) / (2 * 3 * 3)) < 0.0001);
    EXPECT_TRUE(norm(mov.linearAcceleration(0) - Vector2d(872, 1744) / (3 * 4 * 4)) < 0.0001);

    EXPECT_TRUE(std::fabs((mov.angularAcceleration(1) - ContinuousAngle(68) / (2 * 3 * 3)).value()) < 0.0001);
    EXPECT_TRUE(std::fabs((mov.angularAcceleration(0) - ContinuousAngle(2459) / (3 * 4 * 4)).value()) < 0.0001);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
