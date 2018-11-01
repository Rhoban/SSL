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
    // google::protobuf::util::MessageToJsonString(robot, &json_data);
    json_data = "TMP robot: bug version protobuf";
    out <<  json_data;
    return out;

}


std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionBall & ball
){
    std::string json_data;
    // google::protobuf::util::MessageToJsonString(ball, &json_data);
    json_data = "TMP ball: bug version protobuf";
    out << json_data;
    return out;
}



std::ostream& operator<<(
    std::ostream& out, const SSL_DetectionFrame & detection
){
    std::string json_data;
    // google::protobuf::util::MessageToJsonString(detection, &json_data);
    json_data = "TMP detection: bug version protobuf";
    out << "DEBUT" <<json_data << "FIN";
    return out;
}


//};
//};
