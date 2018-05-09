#include <gtest/gtest.h>

#include "robot_control.h"
#include <math/matrix2d.h>


class TestRobotControl : public RobotControl {
    public:
    double get_dt() const {
        return 2.0;
    };
    PidControl no_limited_control(
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

        double translation_velocity_limit = 6;
        double rotation_velocity_limit = 4;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit
        );

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( limited.velocity_translation == Vector2d(3.0, 4.0) );
        EXPECT_TRUE( limited.velocity_rotation == ContinuousAngle(3) );
    }
    {
        TestRobotControl robot_control;

        double translation_velocity_limit = 4;
        double rotation_velocity_limit = 2;

        robot_control.set_limits(
            translation_velocity_limit, rotation_velocity_limit
        );

        DEBUG( "vel : " << robot_control.no_limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        ) );
        DEBUG( "lim vel : " << robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        ) );

        PidControl limited = robot_control.limited_control(
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
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
