#include <gtest/gtest.h>

#include <debug.h>
#include "MovementSample.h"
#include <math.h>

using namespace rhoban_geometry;

TEST(test_MovementSample, position_sample){
    {
        RhobanSSL::PositionSample pos;
        EXPECT_TRUE( pos.time == 0 );
        EXPECT_TRUE( pos.linear_position == Point(0.0, 0.0) );
        EXPECT_TRUE( pos.angular_position == ContinuousAngle(0.0) );
    }

    {
        RhobanSSL::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));
        EXPECT_TRUE( pos.time == 3.0 );
        EXPECT_TRUE( pos.linear_position == Point(4.0, 5.0) );
        EXPECT_TRUE( pos.angular_position == ContinuousAngle(6.0) );
    }

    {
        RhobanSSL::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));
        
        std::ostringstream s1;
        s1 << pos;

        EXPECT_TRUE( "(t=3, lin={4,5}, ang=6)" == s1.str() ); 
    }
}

TEST(test_MovementSample, some_use_cases){
    {
        RhobanSSL::MovementSample mov;
        EXPECT_TRUE( mov.size() == 0 );
    }
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) );
        mov.insert( RhobanSSL::PositionSample( 2, Point(8,16), ContinuousAngle(32) ) );
        mov.insert( RhobanSSL::PositionSample( 3, Point(27,54), ContinuousAngle(108) ) );

        EXPECT_TRUE( mov.linear_position(2) == Point(1,2) );
        EXPECT_TRUE( mov.linear_position(1) == Point(8,16) );
        EXPECT_TRUE( mov.linear_position(0) == Point(27,54) );

        EXPECT_TRUE( mov.angular_position(2) == ContinuousAngle(4) );
        EXPECT_TRUE( mov.angular_position(1) == ContinuousAngle(32) );
        EXPECT_TRUE( mov.angular_position(0) == ContinuousAngle(108) );

        EXPECT_TRUE( mov.linear_velocity(1) == Point(7,14) );
        EXPECT_TRUE( mov.linear_velocity(0) == Point(19,38) );

        EXPECT_TRUE( mov.angular_velocity(1) == ContinuousAngle(28) );
        EXPECT_TRUE( mov.angular_velocity(0) == ContinuousAngle(76) );
        
        EXPECT_TRUE( mov.linear_acceleration(0) == Vector2d(12,24) );

        EXPECT_TRUE( mov.angular_acceleration(0) == ContinuousAngle(48) );
    }
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) );
        mov.insert( RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) );
        mov.insert( RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) );

        EXPECT_TRUE( mov.linear_position(2) == Point(1,2) );
        EXPECT_TRUE( mov.linear_position(1) == Point(8,16) );
        EXPECT_TRUE( mov.linear_position(0) == Point(27,54) );

        EXPECT_TRUE( mov.angular_position(2) == ContinuousAngle(4) );
        EXPECT_TRUE( mov.angular_position(1) == ContinuousAngle(32) );
        EXPECT_TRUE( mov.angular_position(0) == ContinuousAngle(108) );

        EXPECT_TRUE( mov.linear_velocity(1) == Point(7,14)/2 );
        EXPECT_TRUE( mov.linear_velocity(0) == Point(19,38)/3 );

        EXPECT_TRUE( mov.angular_velocity(1) == ContinuousAngle(28)/2 );
        EXPECT_TRUE( mov.angular_velocity(0) == ContinuousAngle(76)/3 );
        
        EXPECT_TRUE( 
            norm( mov.linear_acceleration(0) - Vector2d(17,34)/(2*3*3) )
            < 0.0001 
        );

        EXPECT_TRUE(
            std::fabs( 
                ( 
                    mov.angular_acceleration(0) - ContinuousAngle(68)/(2*3*3)
                ).value()
            ) < 0.0001
        );
    }
    {
        RhobanSSL::MovementSample mov(4);
        mov.insert( RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) );
        mov.insert( RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) );
        mov.insert( RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) );
        mov.insert(
            RhobanSSL::PositionSample( 10, Point(343,686), ContinuousAngle(1029) )
        );

        EXPECT_TRUE( mov.linear_position(3) == Point(1,2) );
        EXPECT_TRUE( mov.linear_position(2) == Point(8,16) );
        EXPECT_TRUE( mov.linear_position(1) == Point(27,54) );
        EXPECT_TRUE( mov.linear_position(0) == Point(343,686) );

        EXPECT_TRUE( mov.angular_position(3) == ContinuousAngle(4) );
        EXPECT_TRUE( mov.angular_position(2) == ContinuousAngle(32) );
        EXPECT_TRUE( mov.angular_position(1) == ContinuousAngle(108) );
        EXPECT_TRUE( mov.angular_position(0) == ContinuousAngle(1029) );

        EXPECT_TRUE( mov.linear_velocity(2) == Point(7,14)/2 );
        EXPECT_TRUE( mov.linear_velocity(1) == Point(19,38)/3 );
        EXPECT_TRUE( mov.linear_velocity(0) == Point(316,632)/4 );

        EXPECT_TRUE( mov.angular_velocity(2) == ContinuousAngle(28)/2 );
        EXPECT_TRUE( mov.angular_velocity(1) == ContinuousAngle(76)/3 );
        EXPECT_TRUE( mov.angular_velocity(0) == ContinuousAngle(921)/4 );
        
        EXPECT_TRUE( 
            norm( mov.linear_acceleration(1) - Vector2d(17,34)/(2*3*3) )
            < 0.0001 
        );
        EXPECT_TRUE( 
            norm( mov.linear_acceleration(0) - Vector2d(872,1744)/(3*4*4) )
            < 0.0001 
        );

        EXPECT_TRUE(
            std::fabs( 
                ( 
                    mov.angular_acceleration(1) - ContinuousAngle(68)/(2*3*3)
                ).value()
            ) < 0.0001
        );
        EXPECT_TRUE(
            std::fabs( 
                ( 
                    mov.angular_acceleration(0) - ContinuousAngle(2459)/(3*4*4)
                ).value()
            ) < 0.0001
        );
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
