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
    std::string json_data;

    #if GOOGLE_PROTOBUF_MIN_LIBRARY_VERSION >= 3000000
        google::protobuf::util::MessageToJsonString(robot, &json_data);
    #else
        json_data = "Upgrade your protobuf version to a version greater than 3.0.0 to print json_data";
    #endif

    
    out <<  json_data;
    return out;

}


std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionBall & ball
){
    std::string json_data;
    #if GOOGLE_PROTOBUF_MIN_LIBRARY_VERSION >= 3000000
        google::protobuf::util::MessageToJsonString(ball, &json_data);
    #else
        json_data = "Upgrade your protobuf version to a version greater than 3.0.0 to print json_data";
    #endif
    out << json_data;
    return out;
}



std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionFrame & detection
){
    std::string json_data;
    #if GOOGLE_PROTOBUF_MIN_LIBRARY_VERSION >= 3000000
        google::protobuf::util::MessageToJsonString(detection, &json_data);
    #else
        json_data = "Upgrade your protobuf version to a version greater than 3.0.0 to print json_data";
    #endif
    out << "DEBUT" <<json_data << "FIN";
    return out;
}


//};
//};
