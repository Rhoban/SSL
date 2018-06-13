#include "robot_position_filter.h"
    
namespace RhobanSSL {
namespace Vision {

std::pair<
    rhoban_geometry::Point,
    ContinuousAngle
>
Robot_position_filter::average_filter(
    int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally, 
    const std::map<int, SSL_DetectionFrame> & camera_detections,
    bool & orientation_is_defined, const Vision::VisionData & old_vision_data
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
            if(
                robot.has_robot_id()
                and (
                    robot.robot_id() == static_cast<unsigned int>( robot_id ) 
                )
            ){
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

std::pair<
    rhoban_geometry::Point,
    ContinuousAngle
>
Robot_position_filter::exponential_degression_filter(
    int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally, 
    const std::map<int, SSL_DetectionFrame> & camera_detections,
    bool & orientation_is_defined, const Vision::VisionData & old_vision_data
){
    double n_linear = 0;
    double n_angular = 0;
    rhoban_geometry::Point linear_average (0.0, 0.0);
    ContinuousAngle angular_average (0.0);
    for( const std::pair<int, SSL_DetectionFrame> & elem : camera_detections ){
        //int camera_id = elem.first;
        const SSL_DetectionFrame & detection = elem.second;
      
        const google::protobuf::RepeatedPtrField<SSL_DetectionRobot> * robots;
        const MovementSample & old_robot_movement = old_vision_data.robots.at(
            ally ? Vision::Team::Ally : Vision::Team::Opponent  
        ).at( robot_id ).movement;
        if(team_color == Ai::Team::Yellow){
            robots = &detection.robots_yellow();
        }else{
            robots = &detection.robots_blue();
        }
        for( auto robot : *robots ){
            if(
                robot.has_robot_id()
                and (
                    robot.robot_id() == static_cast<unsigned int>(robot_id)
                )
            ){
                double x = old_robot_movement.linear_position().getX();
                double alpha = .5;
                double coef = (
                    (
                        (x<0 and robot.x()/1000.0<0)
                        or
                        (x>0 and robot.x()/1000.0>0)
                    )?
                    1 - std::exp( - alpha * std::fabs(x) )
                    :
                    std::exp( - alpha * std::fabs(x) )
                );
                linear_average += (
                    rhoban_geometry::Point(
                        robot.x()/1000.0, robot.y()/1000.0
                    ) * coef
                );
                n_linear += coef;
                if( robot.has_orientation() ){
                    angular_average += (
                        ContinuousAngle(
                            robot.orientation()
                        ) * coef
                    );
                    n_angular += coef;
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

std::pair<
    rhoban_geometry::Point,
    ContinuousAngle
> Robot_position_filter::no_filter(
    int robot_id, const SSL_DetectionRobot & robotFrame, Ai::Team team_color, bool ally,
    const std::map<int, SSL_DetectionFrame> & camera_detections,
    bool & orientation_is_defined, const Vision::VisionData & old_vision_data
){
    orientation_is_defined = robotFrame.has_orientation();
    return {
        rhoban_geometry::Point(robotFrame.x()/1000.0, robotFrame.y()/1000.0), 
        robotFrame.has_orientation() ? ContinuousAngle( robotFrame.orientation() ) : ContinuousAngle(0.0)
    };
}


}
}
