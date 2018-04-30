#include <gtest/gtest.h>

#include "robot_control.h"
#include <math/matrix2d.h>

TEST(test_robot_control, absolute_to_relative_control){
    {
        // No velocity translation 
        // No velocity rotation
        Vector2d velocity_translation(0.0, 0.0);
        ContinuousAngle velocity_rotation(0.0);
        PidControl absolute_control(
            velocity_translation, velocity_rotation
        );
        Vector2d robot_position(1.0,2.0);
        ContinuousAngle robot_orientation(3.0);
        double dt = 1.0;
        
        PidControl control = RobotControl::absolute_to_relative_control(
            absolute_control, robot_position, robot_orientation, dt
        );

        EXPECT_TRUE(control.velocity_translation == Vector2d(0.0,0.0));
        EXPECT_TRUE(control.velocity_rotation == ContinuousAngle(0.0) );
    }
    {
        // No velocity translation 
        Vector2d velocity_translation(0.0, 0.0);
        ContinuousAngle velocity_rotation(2.0);
        PidControl absolute_control(
            velocity_translation, velocity_rotation
        );
        Vector2d robot_position(1.0,2.0);
        ContinuousAngle robot_orientation(3.0);
        double dt = 1.0;
        
        PidControl control = RobotControl::absolute_to_relative_control(
            absolute_control, robot_position, robot_orientation, dt
        );

        EXPECT_TRUE(control.velocity_translation == Vector2d(0.0,0.0));
        EXPECT_TRUE(control.velocity_rotation == ContinuousAngle(2.0) );
    }
    {
        // No velocity rotation
        Vector2d velocity_translation(5.0, 6.0);
        ContinuousAngle velocity_rotation(0.0);
        PidControl absolute_control(
            velocity_translation, velocity_rotation
        );
        Vector2d robot_position(1.0,2.0);
        ContinuousAngle robot_orientation(3.0);
        double dt = 1.0;
        
        PidControl control = RobotControl::absolute_to_relative_control(
            absolute_control, robot_position, robot_orientation, dt
        );
        
        Matrix2d rotation_matrix(
            std::cos(robot_orientation.value()), std::sin(robot_orientation.value()),
            -std::sin(robot_orientation.value()), std::cos(robot_orientation.value())
        );
        Vector2d v = rotation_matrix * velocity_translation;

        EXPECT_TRUE( norm_2( control.velocity_translation - v ) < 0.001 );
        EXPECT_TRUE(control.velocity_rotation == ContinuousAngle(0.0) );
    }
    {
        Vector2d velocity_translation(1.0, 2.0);
        ContinuousAngle velocity_rotation(3.0);
        PidControl absolute_control(
            velocity_translation, velocity_rotation
        );
        Vector2d robot_position(4.0,5.0);
        ContinuousAngle robot_orientation(6.0);
        double dt = 7.0;
        
        PidControl control = RobotControl::absolute_to_relative_control(
            absolute_control, robot_position, robot_orientation, dt
        );

        ContinuousAngle omega = velocity_rotation;
        
        ContinuousAngle theta_t_dt = omega * dt + robot_orientation;
        ContinuousAngle theta_t = omega * 0.0 + robot_orientation;

        Matrix2d matrix_t_dt(
            std::sin(theta_t_dt.value()), std::cos(theta_t_dt.value()),
            - std::cos(theta_t_dt.value()), std::sin(theta_t_dt.value())
        );
        Matrix2d matrix_t(
            std::sin(theta_t.value()), std::cos(theta_t.value()),
            - std::cos(theta_t.value()), std::sin(theta_t.value())
        );

        Vector2d VC = (
            inverse( matrix_t_dt - matrix_t )
        ) * velocity_translation * omega.value() * dt;

        EXPECT_TRUE(
            norm_2( control.velocity_translation - VC ) < 0.001
        );
        EXPECT_TRUE(control.velocity_rotation == ContinuousAngle(3.0) );
    }
}

class TestRobotControl : public RobotControl {
    public:
    double get_dt() const {
        return 2.0;
    };
    PidControl absolute_control_in_absolute_frame(
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

        PidControl limited = robot_control.limited_control(
            Vector2d(0.0, 0.0), ContinuousAngle(0.0)
        );

        EXPECT_TRUE( 
            std::fabs( 
                limited.velocity_translation.norm() - 
                translation_velocity_limit
            ) < 0.0001
        );
        EXPECT_TRUE(
            std::fabs(
                limited.velocity_rotation.value() - rotation_velocity_limit 
            ) < 0.0001 
        );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
