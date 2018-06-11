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

#include "robot_control.h"
#include <math/matrix2d.h>


class TestRobotControl : public RobotControl {
    public:
    virtual double get_dt() const {
        return 2.0;
    };
    virtual PidControl no_limited_control(
        const Vector2d & robot_position, 
        const ContinuousAngle & robot_orientation
    ) const {
        return PidControl(
            Vector2d(3.0, 4.0), 
            ContinuousAngle(3)
        );
    };
};

TEST(test_robot_control, limit_control){

    {
        TestRobotControl robot_control;

        double translation_velocity_limit = -1.0;
        double rotation_velocity_limit = -1.0;
        double translation_acceleration_limit = -1.0;
        double rotation_acceleration_limit = -1.0;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( limited.velocity_translation == Vector2d(3.0, 4.0) );
        EXPECT_TRUE( limited.velocity_rotation == ContinuousAngle(3) );
    }

    {
        TestRobotControl robot_control;

        double translation_velocity_limit = 6;
        double rotation_velocity_limit = 4;
        double translation_acceleration_limit = 100000;
        double rotation_acceleration_limit = 100000;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( limited.velocity_translation == Vector2d(3.0, 4.0) );
        EXPECT_TRUE( limited.velocity_rotation == ContinuousAngle(3) );
    }

    {
        TestRobotControl robot_control;

        double translation_velocity_limit = 6;
        double rotation_velocity_limit = 4;
        double translation_acceleration_limit = -1.0;
        double rotation_acceleration_limit = -1.0;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( limited.velocity_translation == Vector2d(3.0, 4.0) );
        EXPECT_TRUE( limited.velocity_rotation == ContinuousAngle(3) );
    }

    {
        TestRobotControl robot_control;

        double translation_velocity_limit = 4;
        double rotation_velocity_limit = 2;
        double translation_acceleration_limit = 100000;
        double rotation_acceleration_limit = 100000;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( 
            std::fabs( 
                limited.velocity_translation.norm() - 
                translation_velocity_limit*RobotControl::security_margin
            ) < 0.0001
        );
        EXPECT_TRUE(
            std::fabs(
                limited.velocity_rotation.value() - rotation_velocity_limit*RobotControl::security_margin 
            ) < 0.0001 
        );
    }
    {
        TestRobotControl robot_control;

        double dt = robot_control.get_dt();

        double translation_velocity_limit = -1;
        double rotation_velocity_limit = -1;
        double translation_acceleration_limit = 4;
        double rotation_acceleration_limit = 2;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );


        Vector2d v( 9.0, 12.0 );
        ContinuousAngle a(8.0);
        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            v, a
        );

        EXPECT_TRUE( 
            std::fabs( 
                limited.velocity_translation.norm() - 
                (v.norm() - translation_acceleration_limit*dt)/RobotControl::security_margin
            ) < 0.0001
        );
        EXPECT_TRUE(
            std::fabs(
                limited.velocity_rotation.value() -
                (a.value() - rotation_acceleration_limit*dt)/RobotControl::security_margin 
            ) < 0.0001 
        );
    }
    {
        TestRobotControl robot_control;

        double dt = robot_control.get_dt();

        double translation_velocity_limit = 1000000;
        double rotation_velocity_limit = 1000000;
        double translation_acceleration_limit = 4;
        double rotation_acceleration_limit = 2;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit,
            translation_acceleration_limit,
            rotation_acceleration_limit
        );


        Vector2d v( 9.0, 12.0 );
        ContinuousAngle a(8.0);
        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0),
            v, a
        );

        EXPECT_TRUE( 
            std::fabs( 
                limited.velocity_translation.norm() - 
                (v.norm() - translation_acceleration_limit*dt)/RobotControl::security_margin
            ) < 0.0001
        );
        EXPECT_TRUE(
            std::fabs(
                limited.velocity_rotation.value() -
                (a.value() - rotation_acceleration_limit*dt)/RobotControl::security_margin 
            ) < 0.0001 
        );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
