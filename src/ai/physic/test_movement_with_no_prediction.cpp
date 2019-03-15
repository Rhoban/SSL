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

#include <gtest/gtest.h>

#include "movement_with_no_prediction.h"

using namespace rhoban_geometry;

TEST(test_movement_with_no_prediction, use_cases)
{
  {
    RhobanSSL::MovementSample mov(3);
    mov.insert(RhobanSSL::PositionSample(1, Point(1, 2), ContinuousAngle(4)));
    mov.insert(RhobanSSL::PositionSample(3, Point(8, 16), ContinuousAngle(32)));
    mov.insert(RhobanSSL::PositionSample(6, Point(27, 54), ContinuousAngle(108)));

    RhobanSSL::Movement_with_no_prediction pred;
    pred.set_sample(mov);

    EXPECT_TRUE(pred.linear_position(6) == Point(27, 54));
    EXPECT_TRUE(pred.linear_position(6.1) == Point(27, 54));
    EXPECT_TRUE(pred.linear_position(6.2) == Point(27, 54));

    EXPECT_TRUE(pred.angular_position(6) == 108);
    EXPECT_TRUE(pred.angular_position(6.1) == 108);
    EXPECT_TRUE(pred.angular_position(6.2) == 108);

    Vector2d vel = Vector2d(27 - 8, 54 - 16) / 3.0;
    EXPECT_TRUE(pred.linear_velocity(6) == vel);
    EXPECT_TRUE(pred.linear_velocity(6.1) == vel);
    EXPECT_TRUE(pred.linear_velocity(6.2) == vel);

    double ang_vel = (108.0 - 32) / (6 - 3.0);
    EXPECT_TRUE(pred.angular_velocity(6) == ang_vel);
    EXPECT_TRUE(pred.angular_velocity(6.1) == ang_vel);
    EXPECT_TRUE(pred.angular_velocity(6.2) == ang_vel);

    Point vel0 = Point(27 - 8, 54 - 16) / 3.0;
    Point vel1 = Point(8 - 1, 16 - 2) / 2.0;

    Point accel = (vel0 - vel1) / 3.0;

    EXPECT_TRUE(pred.linear_acceleration(6) == accel);
    EXPECT_TRUE(pred.linear_acceleration(6.1) == accel);
    EXPECT_TRUE(pred.linear_acceleration(6.2) == accel);

    double ang_vel1 = (32.0 - 4) / (3 - 1.0);
    double ang_vel0 = (108.0 - 32) / (6 - 3.0);
    double ang_accel = (ang_vel0 - ang_vel1) / (6 - 3.0);

    EXPECT_TRUE(pred.angular_acceleration(6) == ang_accel);
    EXPECT_TRUE(pred.angular_acceleration(6.1) == ang_accel);
    EXPECT_TRUE(pred.angular_acceleration(6.2) == ang_accel);
  }
}

TEST(test_movement_with_no_prediction, clone)
{
  {
    RhobanSSL::MovementSample mov(3);
    mov.insert(RhobanSSL::PositionSample(1, Point(1, 2), ContinuousAngle(4)));
    mov.insert(RhobanSSL::PositionSample(3, Point(8, 16), ContinuousAngle(32)));
    mov.insert(RhobanSSL::PositionSample(6, Point(27, 54), ContinuousAngle(108)));

    RhobanSSL::Movement_with_no_prediction pred_tmp;
    RhobanSSL::Movement* pred_ptr = pred_tmp.clone();
    RhobanSSL::Movement& pred = *pred_ptr;

    pred.set_sample(mov);

    EXPECT_TRUE(pred.linear_position(6) == Point(27, 54));
    EXPECT_TRUE(pred.linear_position(6.1) == Point(27, 54));
    EXPECT_TRUE(pred.linear_position(6.2) == Point(27, 54));

    Point vel = Point(27 - 8, 54 - 16) / 3.0;
    EXPECT_TRUE(pred.linear_velocity(6) == vel);
    EXPECT_TRUE(pred.linear_velocity(6.1) == vel);
    EXPECT_TRUE(pred.linear_velocity(6.2) == vel);

    Point vel0 = Point(27 - 8, 54 - 16) / 3.0;
    Point vel1 = Point(8 - 1, 16 - 2) / 2.0;

    Point accel = (vel0 - vel1) / 3.0;

    EXPECT_TRUE(pred.linear_acceleration(6) == accel);
    EXPECT_TRUE(pred.linear_acceleration(6.1) == accel);
    EXPECT_TRUE(pred.linear_acceleration(6.2) == accel);

    delete (pred_ptr);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
