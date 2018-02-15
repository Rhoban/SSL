#include "pid.h"

class PidStatic : public PidController {
    public:
    ContinuousAngle angle;
    Eigen::Vector2d position;
    
    PidStatic(
        const ContinuousAngle & angle, const Eigen::Vector2d & position 
    ):
        angle(angle), position(position)
    { };

    Eigen::Vector2d goal_position( double t ) const {
        return position;
    };

    ContinuousAngle goal_orientation( double t ) const {
        return angle;
    };
}; 

class PidLinear : public PidController {
    public:
    ContinuousAngle origin_angle;
    Eigen::Vector2d origin_position;
    ContinuousAngle angular_velocity;
    Eigen::Vector2d linear_velocity;
    
    PidLinear(
        const Eigen::Vector2d & origin_position, 
        const ContinuousAngle & origin_angle,
        const Eigen::Vector2d & linear_velocity,
        const ContinuousAngle & angular_velocity
    ):
        origin_angle(origin_angle), origin_position(origin_position), 
        angular_velocity(angular_velocity), linear_velocity(linear_velocity)
    { };

    Eigen::Vector2d goal_position( double t ) const {
        return origin_position + linear_velocity*t;
    };

    ContinuousAngle goal_orientation( double t ) const {
        return origin_angle + angular_velocity*t;
    };
}; 

void test_use_cases(){
    {
        PidStatic controller(
            ContinuousAngle(321), 
            Eigen::Vector2d(21.0, 4.0) 
        );
        controller.set_orientation_pid( 0.0, 0.0, 0.0 );
        controller.set_translation_pid( 0.0, 0.0, 0.0 );
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );

        assert( controller.get_dt() == 10.0 );

        assert( controller.is_static() );
        
        controller.set_static(false);
        assert( ! controller.is_static() );
    }
 
}

void test_is_static(){
    {
        PidStatic controller(
            ContinuousAngle(321), 
            Eigen::Vector2d(21.0, 4.0) 
        );
        controller.set_orientation_pid( 1.0, 1.0, 1.0 );
        controller.set_translation_pid( 1.0, 1.0, 1.0 );
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );

        assert( controller.is_static() );
        
        PidControl control = controller.absolute_control_in_absolute_frame(
            Eigen::Vector2d(1.0, 2.0), ContinuousAngle(0.1)
        );
        assert(
            control.velocity_translation == Eigen::Vector2d(0.0, 0.0)
        );
        assert(
            control.velocity_rotation == ContinuousAngle(0.0)
        );

        controller.set_static(false);
        assert( ! controller.is_static() );

        control = controller.absolute_control_in_absolute_frame(
            Eigen::Vector2d(1.0, 2.0), ContinuousAngle(0.1)
        );
        assert(
            control.velocity_translation != Eigen::Vector2d(0.0, 0.0)
        );
        assert(
            control.velocity_rotation != ContinuousAngle(0.0)
        );
    }
 
}

void test_get_time(){
    {
        PidStatic controller(
            ContinuousAngle(321), 
            Eigen::Vector2d(21.0, 4.0) 
        );
        controller.set_orientation_pid( 0.0, 0.0, 0.0 );
        controller.set_translation_pid( 0.0, 0.0, 0.0 );
        
        controller.init_time( 10.0 );
        
        controller.update( 20.0 );
        assert( controller.get_time() == 10.0 );
        
        controller.update( 40.0 );
        assert( controller.get_time() == 30.0 );

        controller.update( 43.0 );
        assert( controller.get_time() == 33.0 );
    }
}


void test_get_dt(){
    {
        PidStatic controller(
            ContinuousAngle(321), 
            Eigen::Vector2d(21.0, 4.0) 
        );
        controller.set_orientation_pid( 0.0, 0.0, 0.0 );
        controller.set_translation_pid( 0.0, 0.0, 0.0 );
        
        controller.init_time( 10.0 );
        
        controller.update( 20.0 );
        assert( controller.get_dt() == 10.0 );
        
        controller.update( 40.0 );
        assert( controller.get_dt() == 20.0 );

        controller.update( 43.0 );
        assert( controller.get_dt() == 3.0 );
    }
}

void test_null_pid(){
    {
        PidStatic controller(
            ContinuousAngle(321), 
            Eigen::Vector2d(21.0, 4.0) 
        );
        controller.set_orientation_pid( 0.0, 0.0, 0.0 );
        controller.set_translation_pid( 0.0, 0.0, 0.0 );
        controller.set_static(false);
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );
        
        PidControl control = controller.absolute_control_in_absolute_frame(
            Eigen::Vector2d(1.0, 2.0), ContinuousAngle(0.1)
        );

        assert(
            control.velocity_translation == Eigen::Vector2d(0.0,0.0)
        );
        assert(
            control.velocity_rotation == ContinuousAngle(0.0)
        );
    } 
    {
        Eigen::Vector2d origin_position(1.0, 2.0);
        ContinuousAngle origin_orientation(3.0); 
        Eigen::Vector2d linear_velocity(4.0, 5.0);
        ContinuousAngle angular_velocity(6.0); 

        PidLinear controller(
            origin_position,
            origin_orientation,
            linear_velocity,
            angular_velocity
        );
        controller.set_orientation_pid( 0.0, 0.0, 0.0 );
        controller.set_translation_pid( 0.0, 0.0, 0.0 );
        controller.set_static(false);
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );
        
        PidControl control = controller.absolute_control_in_absolute_frame(
            Eigen::Vector2d(1.0, 2.0), ContinuousAngle(0.1)
        );

        assert(
            (
                control.velocity_translation - Eigen::Vector2d(4.0,5.0)
            ).norm() < 0.00001    
        );
        assert(
            std::abs(
                ( control.velocity_rotation - ContinuousAngle(6.0) ).value()
            ) < 0.00001
        );
    } 
}

void test_pid(){
    {
        Eigen::Vector2d origin_position(1.0, 2.0);
        ContinuousAngle origin_orientation(3.0); 
        Eigen::Vector2d linear_velocity(4.0, 5.0);
        ContinuousAngle angular_velocity(6.0); 

        PidLinear controller(
            origin_position,
            origin_orientation,
            linear_velocity,
            angular_velocity
        );

        double kp = 1.0, ki = 0.0, kd = 0.0;

        controller.set_orientation_pid( kp, ki, kd );
        controller.set_translation_pid( kp, ki, kd );
        controller.set_static(false);
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );
        
        Eigen::Vector2d current_position(1.0, 2.0);
        ContinuousAngle current_orientation(0.1);
        PidControl control = controller.absolute_control_in_absolute_frame(
            current_position, current_orientation
        );

        Eigen::Vector2d pos = (
            origin_position + linear_velocity * controller.get_time()
        );
        Eigen::Vector2d error_pos = current_position - pos;
        assert(
            (
                control.velocity_translation - (
                    Eigen::Vector2d(4.0,5.0) - 
                    error_pos * kp / controller.get_dt()
                )
            ).norm() < 0.00001    
        );

        ContinuousAngle ori = (
            origin_orientation + (angular_velocity * controller.get_time())
        );
        ContinuousAngle error_ori = current_orientation - ori;
        assert(
            std::abs(
                (
                    control.velocity_rotation - (
                        ContinuousAngle( 6.0 ) - 
                        ( error_ori * kp / controller.get_dt() )
                    )
                ).value() 
            ) < 0.00001    
        );
    } 
    {
        Eigen::Vector2d origin_position(1.0, 2.0);
        ContinuousAngle origin_orientation(3.0); 
        Eigen::Vector2d linear_velocity(4.0, 5.0);
        ContinuousAngle angular_velocity(6.0); 

        PidLinear controller(
            origin_position,
            origin_orientation,
            linear_velocity,
            angular_velocity
        );

        double kp = 0.0, ki = 1.0, kd = 0.0;

        controller.set_orientation_pid( kp, ki, kd );
        controller.set_translation_pid( kp, ki, kd );
        controller.set_static(false);
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );
        
        Eigen::Vector2d current_position(1.0, 2.0);
        ContinuousAngle current_orientation(0.1);
        PidControl control = controller.absolute_control_in_absolute_frame(
            current_position, current_orientation
        );

        Eigen::Vector2d pos = (
            origin_position + linear_velocity * controller.get_time()
        );
        Eigen::Vector2d error_pos = current_position - pos;
        assert(
            (
                control.velocity_translation - (
                    Eigen::Vector2d(4.0,5.0) - 
                    error_pos * ki
                )
            ).norm() < 0.00001    
        );

        ContinuousAngle ori = (
            origin_orientation + (angular_velocity * controller.get_time())
        );
        ContinuousAngle error_ori = current_orientation - ori;
        assert(
            std::abs(
                (
                    control.velocity_rotation - (
                        ContinuousAngle( 6.0 ) - 
                        error_ori * ki
                    )
                ).value() 
            ) < 0.00001    
        );
    } 
    {
        Eigen::Vector2d origin_position(1.0, 2.0);
        ContinuousAngle origin_orientation(3.0); 
        Eigen::Vector2d linear_velocity(4.0, 5.0);
        ContinuousAngle angular_velocity(6.0); 

        PidLinear controller(
            origin_position,
            origin_orientation,
            linear_velocity,
            angular_velocity
        );

        double kp = 0.0, ki = 0.0, kd = 1.0;

        controller.set_orientation_pid( kp, ki, kd );
        controller.set_translation_pid( kp, ki, kd );
        controller.set_static(false);
        
        controller.init_time( 10.0 );
        controller.update( 20.0 );
        
        Eigen::Vector2d current_position(1.0, 2.0);
        ContinuousAngle current_orientation(0.1);
        PidControl control = controller.absolute_control_in_absolute_frame(
            current_position, current_orientation
        );

        Eigen::Vector2d pos = (
            origin_position + linear_velocity * controller.get_time()
        );
        Eigen::Vector2d error_pos = current_position - pos;
        assert(
            (
                control.velocity_translation - (
                    Eigen::Vector2d(4.0,5.0) - 
                    error_pos * kd / (controller.get_dt()*controller.get_dt())
                )
            ).norm() < 0.00001    
        );

        ContinuousAngle ori = (
            origin_orientation + (angular_velocity * controller.get_time())
        );
        ContinuousAngle error_ori = current_orientation - ori;
        assert(
            std::abs(
                (
                    control.velocity_rotation - (
                        ContinuousAngle( 6.0 ) - 
                        ( error_ori * kd /( controller.get_dt()*controller.get_dt()) )
                    )
                ).value() 
            ) < 0.00001    
        );
    } 
}

int main(){
        
    test_get_dt();
    test_get_time();    
    test_is_static();

    test_null_pid();
    test_pid();

    return 0;
}
