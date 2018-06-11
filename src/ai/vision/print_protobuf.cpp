/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 TO COMPLETE -> Gregwar

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

#include "print_protobuf.h"

//namespace RhobanSSL {
//namespace vision {

std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionRobot & robot
){
    out 
        << "confidence : " << robot.confidence()
        << std::endl
    ;
    if(robot.has_robot_id() ){
        out
            << "robot id : " << robot.robot_id()
            << std::endl
        ;
    }
    out
        << "x : " << robot.x()/1000.0
        << std::endl
        << "y : " << robot.y()/1000.0
        << std::endl
        << "orientation : " << robot.orientation()
        << std::endl
        << "pixel_x : " << robot.pixel_x()/1000.0
        << std::endl
        << "pixel_y : " << robot.pixel_y()/1000.0
        << std::endl
    ;
    if( robot.has_height() ){
        out 
            << robot.height()
            << std::endl
        ;
    }
    return out;
    
}


std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionBall & ball
){
    out 
        << "confidence : " << ball.confidence()
        << std::endl
    ;
    if( ball.has_area() ){
        out
            << "area : " << ball.area()
            << std::endl
        ;
    }
    out
        << "x : " << ball.x()/1000.0
        << std::endl
        << "y : " << ball.y()/1000.0
        << std::endl
    ;
    if( ball.has_z() ){
        out
            << "z : " << ball.z()/1000.0
            << std::endl
        ;
    }
    out
        << "pixel_x : " << ball.pixel_x()/1000.0
        << std::endl
        << "pixel_y : " << ball.pixel_y()/1000.0
        << std::endl
    ;
    return out;
}



std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionFrame & detection 
){
    out 
        << "frame_number : " << detection.frame_number()
        << std::endl
        << "t_capture : " << detection.t_capture()
        << std::endl
        << "t_send : " << detection.t_sent()
        << std::endl
        << "camera id : " << detection.camera_id()
        << std::endl
        << std::endl
        << "BALLS : " 
        << std::endl
    ;
    for( int i=0; i< detection.balls().size(); i++ ){
        out << " ----------- " << std::endl;
        out << detection.balls().Get(i);
    }
    out << std::endl;
    out
        << "YELLOW ROBOTS : " 
        << std::endl
    ;
    for( int i=0; i< detection.robots_yellow().size(); i++ ){
        out << " ----------- " << std::endl;
        out << detection.robots_yellow().Get(i);
    }
    out << std::endl;
    out
        << "BLUE ROBOTS : " 
        << std::endl
    ;
    for( int i=0; i< detection.robots_blue().size(); i++ ){
        out << " ----------- " << std::endl;
        out << detection.robots_blue().Get(i);
    }
    out
        << std::endl
        << std::endl
    ;
    return out;
}


//};
//};


