#include <debug.h>
#include "MovementSample.h"
#include <math.h>

void test_position_sample(){
    {
        RhobanSSL::PositionSample pos;
        assert( pos.time == 0 );
        assert( pos.linear_position == Point(0.0, 0.0) );
        assert( pos.angular_position == ContinuousAngle(0.0) );
    }

    {
        RhobanSSL::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));
        assert( pos.time == 3.0 );
        assert( pos.linear_position == Point(4.0, 5.0) );
        assert( pos.angular_position == ContinuousAngle(6.0) );
    }

    {
        RhobanSSL::PositionSample pos(3.0, Point(4.0, 5.0), ContinuousAngle(6.0));
        
        std::ostringstream s1;
        s1 << pos;

        assert( "(t=3, lin={4,5}, ang=6)" == s1.str() ); 
    }
}

void test_MovementSample(){
    {
        RhobanSSL::MovementSample mov;
        assert( mov.size() == 0 );
    }
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) );
        mov.insert( RhobanSSL::PositionSample( 2, Point(8,16), ContinuousAngle(32) ) );
        mov.insert( RhobanSSL::PositionSample( 3, Point(27,54), ContinuousAngle(108) ) );

        assert( mov.linear_position(2) == Point(1,2) );
        assert( mov.linear_position(1) == Point(8,16) );
        assert( mov.linear_position(0) == Point(27,54) );

        assert( mov.angular_position(2) == ContinuousAngle(4) );
        assert( mov.angular_position(1) == ContinuousAngle(32) );
        assert( mov.angular_position(0) == ContinuousAngle(108) );

        assert( mov.linear_velocity(1) == Point(7,14) );
        assert( mov.linear_velocity(0) == Point(19,38) );

        assert( mov.angular_velocity(1) == ContinuousAngle(28) );
        assert( mov.angular_velocity(0) == ContinuousAngle(76) );
        
        assert( mov.linear_acceleration(0) == Point(12,24) );

        assert( mov.angular_acceleration(0) == ContinuousAngle(48) );
    }
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) );
        mov.insert( RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) );
        mov.insert( RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) );

        assert( mov.linear_position(2) == Point(1,2) );
        assert( mov.linear_position(1) == Point(8,16) );
        assert( mov.linear_position(0) == Point(27,54) );

        assert( mov.angular_position(2) == ContinuousAngle(4) );
        assert( mov.angular_position(1) == ContinuousAngle(32) );
        assert( mov.angular_position(0) == ContinuousAngle(108) );

        assert( mov.linear_velocity(1) == Point(7,14)/2 );
        assert( mov.linear_velocity(0) == Point(19,38)/3 );

        assert( mov.angular_velocity(1) == ContinuousAngle(28)/2 );
        assert( mov.angular_velocity(0) == ContinuousAngle(76)/3 );
        
        assert( 
            ( mov.linear_acceleration(0) - Point(17,34)/(2*3*3) ).getDist(
                Point(0.0,0.0)
            ) 
            < 0.0001 
        );

        assert(
            std::abs( 
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

        assert( mov.linear_position(3) == Point(1,2) );
        assert( mov.linear_position(2) == Point(8,16) );
        assert( mov.linear_position(1) == Point(27,54) );
        assert( mov.linear_position(0) == Point(343,686) );

        assert( mov.angular_position(3) == ContinuousAngle(4) );
        assert( mov.angular_position(2) == ContinuousAngle(32) );
        assert( mov.angular_position(1) == ContinuousAngle(108) );
        assert( mov.angular_position(0) == ContinuousAngle(1029) );

        assert( mov.linear_velocity(2) == Point(7,14)/2 );
        assert( mov.linear_velocity(1) == Point(19,38)/3 );
        assert( mov.linear_velocity(0) == Point(316,632)/4 );

        assert( mov.angular_velocity(2) == ContinuousAngle(28)/2 );
        assert( mov.angular_velocity(1) == ContinuousAngle(76)/3 );
        assert( mov.angular_velocity(0) == ContinuousAngle(921)/4 );
        
        assert( 
            ( mov.linear_acceleration(1) - Point(17,34)/(2*3*3) ).getDist(
                Point(0.0,0.0)
            ) 
            < 0.0001 
        );
        assert( 
            ( mov.linear_acceleration(0) - Point(872,1744)/(3*4*4) ).getDist(
                Point(0.0,0.0)
            ) 
            < 0.0001 
        );

        assert(
            std::abs( 
                ( 
                    mov.angular_acceleration(1) - ContinuousAngle(68)/(2*3*3)
                ).value()
            ) < 0.0001
        );
        assert(
            std::abs( 
                ( 
                    mov.angular_acceleration(0) - ContinuousAngle(2459)/(3*4*4)
                ).value()
            ) < 0.0001
        );
    }
}

int main(){
    test_position_sample();
    test_MovementSample();
}
