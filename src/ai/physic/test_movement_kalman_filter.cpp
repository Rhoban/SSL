#include <gtest/gtest.h>

#include "movement_kalman_filter.h"
#include <cmath>

using namespace rhoban_geometry;

TEST(test_movement_kalman_filter, use_cases){
    {
        RhobanSSL::MovementSample video_mov(3);
        RhobanSSL::MovementSample odom_mov(9);

        video_mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) 
        );
        video_mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) 
        );
        video_mov.insert( 
            RhobanSSL::PositionSample( 5, Point(27,54), ContinuousAngle(108) ) 
        );

        odom_mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(10) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 1.5, Point(1,2), ContinuousAngle(15) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 2, Point(8,16), ContinuousAngle(20) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 2.5, Point(27,54), ContinuousAngle(30) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(40) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 3.5, Point(1,2), ContinuousAngle(10) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 4, Point(8,16), ContinuousAngle(40) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 4.5, Point(27,54), ContinuousAngle(90) ) 
        );
        odom_mov.insert( 
            RhobanSSL::PositionSample( 5, Point(27,54), ContinuousAngle(90) ) 
        );

        RhobanSSL::Movement_kalman_filter kal;
        kal.set_sample( mov );

        Point pos(27,54);
        EXPECT_TRUE( kal.linear_position(6) == pos );

        ContinuousAngle ang( 108 );
        EXPECT_TRUE( kal.angular_position(6) == ang );

        Point vel = Point(27-8,54-16)/3.0;
        EXPECT_TRUE( kal.linear_velocity(6) == vel );
       
        double ang_vel = (108.0-32)/(6-3.0);
        EXPECT_TRUE( kal.angular_velocity(6) == ang_vel );

 
        Point vel0 = Point(27-8,54-16)/3.0;
        Point vel1 = Point(8-1,16-2)/2.0;

        Point accel = (vel0 - vel1)/3.0;

        EXPECT_TRUE( kal.linear_acceleration(6) == accel );

        double ang_vel1 = (32.0-4)/(3-1.0);
        double ang_vel0 = (108.0-32)/(6-3.0);
        double ang_accel = (ang_vel0 - ang_vel1)/(6-3.0);

        EXPECT_TRUE( kal.angular_acceleration(6) == ang_accel );

        double step = 0.1;
        double t = 0;
        for( int i=0; i<100; i++ ){
            EXPECT_TRUE( 
                norm(
                    kal.linear_position(6+t) - (
                        pos + vel*t + accel*t*t/2.0 
                    )
                ) < 0.00001
            );
            EXPECT_TRUE(
                std::fabs(
                    (
                        kal.angular_position(6+t) - (
                            ang + ang_vel*t + ang_accel*t*t/2.0 
                        )
                    ).value()
                ) < 0.00001
            );

            EXPECT_TRUE( kal.linear_acceleration(6+t) == accel );
            EXPECT_TRUE( kal.angular_acceleration(6+t) == ang_accel );
            t+= step;
        }

    }
}