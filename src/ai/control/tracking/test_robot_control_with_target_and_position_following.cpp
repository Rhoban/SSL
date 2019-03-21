/*
    This file is part of SSL.

    Copyright 2019 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
#include "robot_control_with_target_tracking_and_position_following.h"


TEST(test_controls_values_of_a_movement, robot_doesnt_need_to_change_its_angular_position)
{
  rhoban_ssl::robot_control::tracking::RobotControlWithTargetTrackingAndPositionFollowing robot_control;

  //
  rhoban_geometry::Point linear_position_of_the_robot( 0.0, 0.0);
  ContinuousAngle angular_position_of_the_robot(0.0);
  Vector2d linear_velocity_of_the_robot(0.0,0.0);

  //the robot tracks a target at`Point(1.0,0.0)` which is motionless.
  rhoban_geometry::Point linear_position_of_the_target( 1.0, 0.0);
  rhoban_geometry::Point linear_velocity_of_the_target( 0.0, 0.0);


  Vector2d direction = linear_position_of_the_target - linear_position_of_the_robot;
  ContinuousAngle angle_between_robot_and_target_at_start = vector2angle(direction);

  //At the beginning, the robot is in front of the point
  EXPECT_TRUE( angular_position_of_the_robot == angle_between_robot_and_target_at_start);

  //the robot go to the left
  rhoban_geometry::Point destination_of_the_robot( -4.0, 0.0);


  robot_control.initTime(100, 60);
  robot_control.setGoal(destination_of_the_robot,  linear_position_of_the_target, linear_velocity_of_the_target);

  robot_control.update(100, linear_position_of_the_robot, linear_velocity_of_the_robot, angular_position_of_the_robot);
  Control ctrl = robot_control.getLimitedControl();

 // EXPECT_TRUE(ctrl.linear_velocity)
}



TEST(test_robot_control_with_target_tracking_and_position_following, qsdfsdf)
{

}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


