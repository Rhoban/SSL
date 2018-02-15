#include "movement_with_no_prediction.h"


void test_use_cases(){
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) 
        );

        RhobanSSL::Movement_with_no_prediction pred;
        pred.set_sample( mov );

        assert( pred.linear_position(6) == Point(27,54) );
        assert( pred.linear_position(6.1) == Point(27,54) );
        assert( pred.linear_position(6.2) == Point(27,54) );

        assert( pred.angular_position(6) == 108 );
        assert( pred.angular_position(6.1) == 108 );
        assert( pred.angular_position(6.2) == 108 );

        Point vel = Point(27-8,54-16)/3.0;
        assert( pred.linear_velocity(6) == vel );
        assert( pred.linear_velocity(6.1) == vel );
        assert( pred.linear_velocity(6.2) == vel );
       
        double ang_vel = (108.0-32)/(6-3.0);
        assert( pred.angular_velocity(6) == ang_vel );
        assert( pred.angular_velocity(6.1) == ang_vel );
        assert( pred.angular_velocity(6.2) == ang_vel );

 
        Point vel0 = Point(27-8,54-16)/3.0;
        Point vel1 = Point(8-1,16-2)/2.0;

        Point accel = (vel0 - vel1)/3.0;

        assert( pred.linear_acceleration(6) == accel );
        assert( pred.linear_acceleration(6.1) == accel );
        assert( pred.linear_acceleration(6.2) == accel );

        double ang_vel1 = (32.0-4)/(3-1.0);
        double ang_vel0 = (108.0-32)/(6-3.0);
        double ang_accel = (ang_vel0 - ang_vel1)/(6-3.0);

        assert( pred.angular_acceleration(6) == ang_accel );
        assert( pred.angular_acceleration(6.1) == ang_accel );
        assert( pred.angular_acceleration(6.2) == ang_accel );
    }
}

void test_clone(){
    {
        RhobanSSL::MovementSample mov(3);
        mov.insert( 
            RhobanSSL::PositionSample( 1, Point(1,2), ContinuousAngle(4) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 3, Point(8,16), ContinuousAngle(32) ) 
        );
        mov.insert( 
            RhobanSSL::PositionSample( 6, Point(27,54), ContinuousAngle(108) ) 
        );

        RhobanSSL::Movement_with_no_prediction pred_tmp;
        RhobanSSL::Movement * pred_ptr = pred_tmp.clone();
        RhobanSSL::Movement & pred = *pred_ptr;
        
        pred.set_sample( mov );

        assert( pred.linear_position(6) == Point(27,54) );
        assert( pred.linear_position(6.1) == Point(27,54) );
        assert( pred.linear_position(6.2) == Point(27,54) );

        Point vel = Point(27-8,54-16)/3.0;
        assert( pred.linear_velocity(6) == vel );
        assert( pred.linear_velocity(6.1) == vel );
        assert( pred.linear_velocity(6.2) == vel );
        
        Point vel0 = Point(27-8,54-16)/3.0;
        Point vel1 = Point(8-1,16-2)/2.0;

        Point accel = (vel0 - vel1)/3.0;

        assert( pred.linear_acceleration(6) == accel );
        assert( pred.linear_acceleration(6.1) == accel );
        assert( pred.linear_acceleration(6.2) == accel );

        delete(pred_ptr);
    }
}

int main(){
    
    test_use_cases();
    test_clone();

    return 0;
}

