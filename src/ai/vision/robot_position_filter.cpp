#include "robot_position_filter.h"
    
namespace RhobanSSL {
namespace vision {

std::pair<
    rhoban_geometry::Point,
    ContinuousAngle
>
Robot_position_filter::average_filter(
    int robot_id, Ai::Team team_color, 
    const std::map<int, SSL_DetectionFrame> & camera_detections,
    bool & orientation_is_defined
){
    int n_linear = 0;
    int n_angular = 0;
    rhoban_geometry::Point linear_average (0.0, 0.0);
    ContinuousAngle angular_average (0.0);
    for( const std::pair<int, SSL_DetectionFrame> & elem : camera_detections ){
        //int camera_id = elem.first;
        const SSL_DetectionFrame & detection = elem.second;
      
        const google::protobuf::RepeatedPtrField<SSL_DetectionRobot> * robots;
        if(team_color == Ai::Team::Yellow){
            robots = &detection.robots_yellow();
        }else{
            robots = &detection.robots_blue();
        }
        for( auto robot : *robots ){
            if( robot.has_robot_id() and robot.robot_id() == robot_id ){
                linear_average += rhoban_geometry::Point(
                    robot.x()/1000.0, robot.y()/1000.0
                );
                n_linear ++;
                if( robot.has_orientation() ){
                    angular_average += ContinuousAngle(
                        robot.orientation()
                    );
                    n_angular ++;
                }
                break;
            }
        }
    }
    if( n_angular == 0 ){
        orientation_is_defined = false;
        return {
            linear_average*(1.0/n_linear),
            ContinuousAngle(0.0) 
        };
    }else{
        orientation_is_defined = true;
        return {
            linear_average*(1.0/n_linear),
            angular_average*(1.0/n_angular)
        };
    }
}

}
}
