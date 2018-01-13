#include <iostream>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "VisionClient.h"

static volatile bool running = true;

void stop(int s)
{
    running = false;
}

int main()
{
    signal(SIGINT, stop);
    RhobanSSL::VisionClient client(true);

    while (running) {
        std::stringstream ss;

        // ss << "Vision infos" << std::endl;

        if (client.hasData()) {
            auto data = client.getData();

            // see messages_robocup_ssl_detection.proto
            auto detection = data.detection();
            ss << "frame: " << detection.frame_number() << std::endl;
            ss << "t_capture: " << detection.t_capture() << std::endl;
            ss << "t_sent: " << detection.t_sent() << std::endl;
            ss << "camera_id: " << detection.camera_id() << std::endl;

            for (auto ball : detection.balls()) {
                ss << "ball:" << std::endl;
                ss << "  confidence: " << ball.confidence() << std::endl;
                ss << "  area: " << ball.area() << std::endl;
                ss << "  x: " << ball.x() << std::endl;
                ss << "  y: " << ball.y() << std::endl;
                ss << "  z: " << ball.z() << std::endl;
            }
            for (auto robot : detection.robots_blue()) {
                ss << "blue robot [";
                ss << "  id: " << robot.robot_id() << ",";
                ss << "  confidence: " << robot.confidence() << ",";
                ss << "  x: " << robot.x() << ",";
                ss << "  y: " << robot.y() << "]" << std::endl;

            }

            // see messages_robocup_ssl_geometry.proto
            auto geometry = data.geometry();
            if (geometry.has_field()) {
                ss << "field:" << std::endl;
                ss << "  field_length: " << geometry.field().field_length() << std::endl;
                ss << "  field_width: " << geometry.field().field_width() << std::endl;
                ss << "  goal_width: " << geometry.field().goal_width() << std::endl;
            }

        } else {
            ss << "*no data*" << std::endl;
        }

        (void)system("clear");
        std::cout << ss.str();
        std::cout << std::flush;
        usleep(30000);
    }
}
