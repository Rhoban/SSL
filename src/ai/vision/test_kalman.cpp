#include "factory.h"
#include "vision_data.h"
#include <cmath>
using namespace rhoban_ssl;
using namespace vision;


Position computeAverage(RobotDetection** robot_views) {

    double cosa = 0.0;
    double sina = 0.0;
    double average_x = 0.0;
    double average_y = 0.0;
    int counter = 0;
    ContinuousAngle average_angle;
    rhoban_geometry::Point average_point;
    Position average_position;

    for(unsigned int i=0; i<ai::Config::NB_CAMERAS; ++i) {
        RobotDetection* robot_view = robot_views[i];
            if (robot_view != nullptr && robot_view->has_id_){

                cosa += cos(robot_view->orientation_);
                sina += sin(robot_view->orientation_);
                average_x += robot_view->x_/1000;
                average_y += robot_view->y_/1000;
                ++counter;
            }
        
    }
    average_angle = ContinuousAngle(atan2(sina, cosa));
    average_point = rhoban_geometry::Point(average_x/(double) counter, average_y/(double) counter);
    average_position = Position(average_point, average_angle);
    return(average_position);
}


void testKalman() {

    RobotDetection* robot_views[ai::Config::NB_CAMERAS];
    for(unsigned int k = 0; k<ai::Config::NB_CAMERAS; ++k){
        robot_views[k] = nullptr;
    }

    double cadence_time = 1.0;
    double previous_time = 0.0;
    double offset_x = 2000.0;
    double offset_y = 0.0;
    double speed_x = 1000.0;
    double speed_y = 0.0;
    double orientation = 0.0;
    double angular_speed = 1.0;
    Position average_position = Position();
    Position true_position = Position(rhoban_geometry::Point(offset_x/1000, offset_y/1000), ContinuousAngle(orientation));
    TimedPosition t = TimedPosition();
    Kalman f = Kalman();

//This one is unused//
    RobotDetection r1 = RobotDetection();
    r1.x_ = offset_x;
    r1.y_ = offset_y;
    r1.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r1.v_y_ = speed_y;
    r1.orientation_ = orientation + 1.0;
    r1.angular_speed_ = angular_speed;
    r1.has_orientation_ = true;
    r1.has_id_ = true;
/////////////////////

    RobotDetection r2 = RobotDetection();
    r2.x_ = offset_x;
    r2.y_ = offset_y;
    r2.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r2.v_y_ = speed_y;
    r2.orientation_ = orientation;
    r2.angular_speed_ = angular_speed;
    r2.has_orientation_ = true;
    r2.has_id_ = true;

    RobotDetection r3 = RobotDetection();
    r3.x_ = offset_x + 50.0;
    r3.y_ = offset_y + 200.0;
    r3.v_x_ = speed_x;
    r3.v_y_ = speed_y;
    r3.orientation_ = orientation + 2.0*M_PI;
    r3.angular_speed_ = angular_speed;
    r3.has_orientation_ = true;
    r3.has_id_ = true;

    RobotDetection r4 = RobotDetection();
    r4.x_ = offset_x - 100.0;
    r4.y_ = offset_y + 100.0;
    r4.v_x_ = speed_x; //will variate btw 0.8 and 1.2
    r4.v_y_ = speed_y;
    r4.orientation_ = orientation + 0.3;
    r4.angular_speed_ = angular_speed;
    r4.has_orientation_ = true;
    r4.has_id_ = true;

    robot_views[0] = &r2;

    robot_views[1] = &r3;

    robot_views[3] = &r4;

    for(int k=0; k<20; ++k){

        printf("New execution starts:\nNow calling Kalman filter... for the %d execution...\n", k);
        printf("Angular speed is 1 rad/s\n");
        printf("Previous position was : ");
        std::cout << t.position_;
        printf("\n");
        printf("Kalman execution ongoing...\n");
        t = f.kalmanFilter(robot_views, cadence_time);

        printf("New position is : ");
        std::cout << t.position_;
        printf(" while true position is ");
        std::cout << true_position;
        printf("and average algorithm yields ");
        //start of barycenter computing
        average_position = computeAverage(robot_views);
        //end of barycenter computing
        std::cout << average_position;
        printf("\n");

        printf("Update of robot_views and cadence time...");
        //update of robot_views
        for(unsigned int i = 0; i<ai::Config::NB_CAMERAS; ++i){
            if(robot_views[i] != nullptr){
                robot_views[i]->x_ = robot_views[i]->x_ + speed_x;
                robot_views[i]->y_ = robot_views[i]->y_ + speed_y;
                robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);

                if(i == 0) {
                    robot_views[i]->v_x_ = speed_x + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                }
                if(i == 3) {
                    robot_views[i]->v_x_ = speed_y + 0.2*cos(cadence_time)*robot_views[i]->v_x_; 
                }
            }
        }
        //true position update
        true_position.linear += rhoban_geometry::Point((cadence_time - previous_time)*speed_x/1000, (cadence_time - previous_time)*speed_y/1000);
        true_position.angular = ContinuousAngle(std::fmod(cadence_time,2.0*M_PI));
        //time update
        previous_time = cadence_time;
        cadence_time += 1.0;
        printf("done...\n\n\n");
    }
}




void testKalmanNullptr() {

    RobotDetection* robot_views[ai::Config::NB_CAMERAS];
    for(unsigned int k = 0; k<ai::Config::NB_CAMERAS; ++k){
        robot_views[k] = nullptr;
    }

    double cadence_time = 1.0;
    double previous_time = 0.0;
    double offset_x = 2000.0;
    double offset_y = 0.0;
    double speed_x = 1000.0;
    double speed_y = 0.0;
    double orientation = 0.0;
    double angular_speed = 1.0;
    Position average_position = Position();
    Position true_position = Position(rhoban_geometry::Point(offset_x/1000, offset_y/1000), ContinuousAngle(orientation));
    TimedPosition t = TimedPosition();
    Kalman f = Kalman();

    for(int k=0; k<20; ++k){

        printf("New execution starts:\nNow calling Kalman filter... for the %d execution...\n", k);
        printf("Angular speed is 1 rad/s\n");
        printf("Previous position was : ");
        std::cout << t.position_;
        printf("\n");
        printf("Kalman execution ongoing...\n");
        t = f.kalmanFilter(robot_views, cadence_time);

        printf("New position is : ");
        std::cout << t.position_;
        printf(" while true position is ");
        std::cout << true_position;
        printf("and average algorithm yields ");
        //start of barycenter computing
        average_position = computeAverage(robot_views);
        //end of barycenter computing
        std::cout << average_position;
        printf("\n");

        printf("Update of robot_views and cadence time...");
        //update of robot_views
        for(unsigned int i = 0; i<ai::Config::NB_CAMERAS; ++i){
            if(robot_views[i] != nullptr){
                robot_views[i]->x_ = robot_views[i]->x_ + speed_x;
                robot_views[i]->y_ = robot_views[i]->y_ + speed_y;
                robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);

                if(i == 0) {
                    robot_views[i]->v_x_ = speed_x + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                }
                if(i == 3) {
                    robot_views[i]->v_x_ = speed_y + 0.2*cos(cadence_time)*robot_views[i]->v_x_; 
                }
            }
        }
        //true position update
        true_position.linear += rhoban_geometry::Point((cadence_time - previous_time)*speed_x/1000, (cadence_time - previous_time)*speed_y/1000);
        true_position.angular = ContinuousAngle(std::fmod(cadence_time,2.0*M_PI));
        //time update
        previous_time = cadence_time;
        cadence_time += 1.0;
        printf("done...\n\n\n");
    }
}

void testKalmanNoId() {

    RobotDetection* robot_views[ai::Config::NB_CAMERAS];
    for(unsigned int k = 0; k<ai::Config::NB_CAMERAS; ++k){
        robot_views[k] = nullptr;
    }

    double cadence_time = 1.0;
    double previous_time = 0.0;
    double offset_x = 2000.0;
    double offset_y = 0.0;
    double speed_x = 1000.0;
    double speed_y = 0.0;
    double orientation = 0.0;
    double angular_speed = 1.0;
    Position average_position = Position();
    Position true_position = Position(rhoban_geometry::Point(offset_x/1000, offset_y/1000), ContinuousAngle(orientation));
    TimedPosition t = TimedPosition();
    Kalman f = Kalman();


//This one is unused//
    RobotDetection r1 = RobotDetection();
    r1.x_ = offset_x;
    r1.y_ = offset_y;
    r1.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r1.v_y_ = speed_y;
    r1.orientation_ = orientation + 1.0;
    r1.angular_speed_ = angular_speed;
    r1.has_orientation_ = true;
    r1.has_id_ = false;
/////////////////////
    RobotDetection r2 = RobotDetection();
    r2.x_ = offset_x;
    r2.y_ = offset_y;
    r2.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r2.v_y_ = speed_y;
    r2.orientation_ = orientation;
    r2.angular_speed_ = angular_speed;
    r2.has_orientation_ = true;
    r2.has_id_ = false;

    RobotDetection r3 = RobotDetection();
    r3.x_ = offset_x + 50.0;
    r3.y_ = offset_y + 200.0;
    r3.v_x_ = speed_x;
    r3.v_y_ = speed_y;
    r3.orientation_ = orientation + 2.0*M_PI;
    r3.angular_speed_ = angular_speed;
    r3.has_orientation_ = true;
    r3.has_id_ = false;
    
    RobotDetection r4 = RobotDetection();
    r4.x_ = offset_x - 100.0;
    r4.y_ = offset_y + 100.0;
    r4.v_x_ = speed_x; //will variate btw 0.8 and 1.2
    r4.v_y_ = speed_y;
    r4.orientation_ = orientation + 0.3;
    r4.angular_speed_ = angular_speed;
    r4.has_orientation_ = true;
    r4.has_id_ = false;


    robot_views[0] = &r2;

    robot_views[1] = &r3;

    robot_views[3] = &r4;


    for(int k=0; k<20; ++k){

        printf("New execution starts:\nNow calling Kalman filter... for the %d execution...\n", k);
        printf("Angular speed is 1 rad/s\n");
        printf("Previous position was : ");
        std::cout << t.position_;
        printf("\n");
        printf("Kalman execution ongoing...\n");
        t = f.kalmanFilter(robot_views, cadence_time);

        printf("New position is : ");
        std::cout << t.position_;
        printf(" while true position is ");
        std::cout << true_position;
        printf("and average algorithm yields ");
        //start of barycenter computing
        average_position = computeAverage(robot_views);
        //end of barycenter computing
        std::cout << average_position;
        printf("\n");

        printf("Update of robot_views and cadence time...");
        //update of robot_views
        for(unsigned int i = 0; i<ai::Config::NB_CAMERAS; ++i){
            if(robot_views[i]!= nullptr){
                robot_views[i]->x_ = robot_views[i]->x_ + speed_x;
                robot_views[i]->y_ = robot_views[i]->y_ + speed_y;
                robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);

                if(i == 0) {
                    robot_views[i]->v_x_ = speed_x + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                }
                if(i == 3) {
                    robot_views[i]->v_x_ = speed_y + 0.2*cos(cadence_time)*robot_views[i]->v_x_; 
                }
            }
        }
        //true position update
        true_position.linear += rhoban_geometry::Point((cadence_time - previous_time)*speed_x/1000, (cadence_time - previous_time)*speed_y/1000);
        true_position.angular = ContinuousAngle(std::fmod(cadence_time,2.0*M_PI));
        //time update
        previous_time = cadence_time;
        cadence_time += 1.0;
        printf("done...\n\n\n");
    }
}

void testKalman2() {

    RobotDetection* robot_views[ai::Config::NB_CAMERAS];
    for(unsigned int k = 0; k<ai::Config::NB_CAMERAS; ++k){
        robot_views[k] = nullptr;
    }
    double cadence_time = 1.0;
    double previous_time = 0.0;
    double offset_x = 2000.0;
    double offset_y = 0.0;
    double speed_x = 1000.0;
    double speed_y = 0.0;
    double orientation = 0.0;
    double angular_speed = 1.0;
    Position average_position = Position();
    Position true_position = Position(rhoban_geometry::Point(offset_x/1000, offset_y/1000), ContinuousAngle(orientation));
    TimedPosition t = TimedPosition();
    Kalman f = Kalman();


//This one is unused//
    RobotDetection r1 = RobotDetection();
    r1.x_ = offset_x;
    r1.y_ = offset_y;
    r1.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r1.v_y_ = speed_y;
    r1.orientation_ = orientation + 1.0;
    r1.angular_speed_ = angular_speed;
    r1.has_orientation_ = true;
    r1.has_id_ = false;
///////////////////////

    RobotDetection r2 = RobotDetection();
    r2.x_ = offset_x; //will variate btw 0.9 and 1.1, with cos
    r2.y_ = offset_y; //will variate btw 0.9 and 1.1, with cos + pi/4
    r2.v_x_ = speed_x; //will variate btw 0.9 and 1.1, with cos
    r2.v_y_ = speed_y;
    r2.orientation_ = orientation;
    r2.angular_speed_ = angular_speed;
    r2.has_orientation_ = true;
    r2.has_id_ = true;

    RobotDetection r3 = RobotDetection();
    r3.x_ = offset_x + 50.0; //will variate btw 0.9 and 1.1, with cos
    r3.y_ = offset_y + 200.0; //will variate btw 0.9 and 1.1, with sin
    r3.v_x_ = speed_x; //will variate btw 0.9 and 1.1, with sin
    r3.v_y_ = speed_y;
    r3.orientation_ = orientation + 2.0*M_PI;
    r3.angular_speed_ = angular_speed;
    r3.has_orientation_ = true;
    r3.has_id_ = true;
    
    RobotDetection r4 = RobotDetection();
    r4.x_ = offset_x - 100.0; //will variate btw 0.9 and 1.1, with cos
    r4.y_ = offset_y + 100.0; //will variate btw 0.8 and 1.2, with sin + pi/4
    r4.v_x_ = speed_x; //will variate btw 0.8 and 1.2, with cos
    r4.v_y_ = speed_y;
    r4.orientation_ = orientation + 0.3;
    r4.angular_speed_ = angular_speed;
    r4.has_orientation_ = true;
    r4.has_id_ = true;


    robot_views[0] = &r2;

    robot_views[1] = &r3;

    robot_views[3] = &r4;


    for(int k=0; k<20; ++k){

        printf("New execution starts:\nNow calling Kalman filter... for the %d execution...\n", k);
        printf("Angular speed is 1 rad/s\n");
        printf("Previous position was : ");
        std::cout << t.position_;
        printf("\n");
        printf("Kalman execution ongoing...\n");
        t = f.kalmanFilter(robot_views, cadence_time);

        printf("New position is : ");
        std::cout << t.position_;
        printf(" while true position is ");
        std::cout << true_position;
        printf("and average algorithm yields ");
        //start of barycenter computing
        average_position = computeAverage(robot_views);
        //end of barycenter computing
        std::cout << average_position;
        printf("\n");

        printf("Update of robot_views and cadence time...");
        //update of robot_views
        for(unsigned int i = 0; i<ai::Config::NB_CAMERAS; ++i){
            if(robot_views[i] != nullptr){
                robot_views[i]->x_ = robot_views[i]->x_ + speed_x;
                robot_views[i]->y_ = robot_views[i]->y_ + speed_y;
                robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);

                if(i == 0) {
                    robot_views[i]->v_x_ = speed_x + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                    robot_views[i]->x_ = robot_views[i]->x_+ 0.1*cos(cadence_time)*robot_views[i]->x_; 
                    robot_views[i]->y_ = robot_views[i]->y_ + 0.1*cos(cadence_time + M_PI/4)*robot_views[i]->y_; 
                }
                if(i == 2){
                    robot_views[i]->v_x_ = speed_x + 0.1*sin(cadence_time)*robot_views[i]->v_x_; 
                    robot_views[i]->x_ = robot_views[i]->x_ + 0.1*cos(cadence_time)*robot_views[i]->x_; 
                    robot_views[i]->y_ = robot_views[i]->y_ + 0.1*sin(cadence_time + M_PI/4)*robot_views[i]->y_; 
                }
                if(i == 3) {
                    robot_views[i]->v_x_ = speed_y + 0.2*cos(cadence_time)*robot_views[i]->v_x_; 
                    robot_views[i]->y_ = robot_views[i]->y_ + 0.2*sin(cadence_time + M_PI/4)*robot_views[i]->y_; 
                    robot_views[i]->x_ = robot_views[i]->x_ + 0.1*cos(cadence_time)*robot_views[i]->x_; 
                }
                if(i == 4){
                    robot_views[i]->v_x_ = speed_y + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                    robot_views[i]->y_ = robot_views[i]->y_ + 0.1*cos(cadence_time + M_PI/4)*robot_views[i]->y_; 
                    robot_views[i]->x_ = robot_views[i]->x_ + 0.2*sin(cadence_time+M_PI/4)*robot_views[i]->x_; 

                }
            }
                
        }
        //true position update
        true_position.linear += rhoban_geometry::Point((cadence_time - previous_time)*speed_x/1000, (cadence_time - previous_time)*speed_y/1000);
        true_position.angular = ContinuousAngle(std::fmod(cadence_time,2.0*M_PI));
        //time update
        previous_time = cadence_time;
        cadence_time += 1.0;
        printf("done...\n\n\n");
    }
}


void testKalmanRobotTeleport() {

    RobotDetection* robot_views[ai::Config::NB_CAMERAS];
    for(unsigned int k = 0; k<ai::Config::NB_CAMERAS; ++k){
        robot_views[k] = nullptr;
    }

    double cadence_time = 1.0;
    double previous_time = 0.0;
    double offset_x = 2000.0;
    double offset_y = 0.0;
    double offset_x2 = -4000.0;
    double offset_y2 = -5000.0;
    double speed_x = 1000.0;
    double speed_y = 0.0;
    double orientation = 0.0;
    double angular_speed = 1.0;
    Position average_position = Position();
    Position true_position = Position(rhoban_geometry::Point(offset_x/1000, offset_y/1000), ContinuousAngle(orientation));
    TimedPosition t = TimedPosition();
    Kalman f = Kalman();


//This one is unused//
    RobotDetection r1 = RobotDetection();
    r1.x_ = offset_x;
    r1.y_ = offset_y;
    r1.v_x_ = speed_x; //will variate btw 0.9 and 1.1
    r1.v_y_ = speed_y;
    r1.orientation_ = orientation + 1.0;
    r1.angular_speed_ = angular_speed;
    r1.has_orientation_ = true;
    r1.has_id_ = false;
/////////////////////

    RobotDetection r2 = RobotDetection();
    r2.x_ = offset_x; //will variate btw 0.9 and 1.1, with cos
    r2.y_ = offset_y; //will variate btw 0.9 and 1.1, with cos + pi/4
    r2.v_x_ = speed_x; //will variate btw 0.9 and 1.1, with cos
    r2.v_y_ = speed_y;
    r2.orientation_ = orientation;
    r2.angular_speed_ = angular_speed;
    r2.has_orientation_ = true;
    r2.has_id_ = true;

    RobotDetection r3 = RobotDetection();
    r3.x_ = offset_x + 50.0; //will variate btw 0.9 and 1.1, with cos
    r3.y_ = offset_y + 200.0; //will variate btw 0.9 and 1.1, with sin
    r3.v_x_ = speed_x; //will variate btw 0.9 and 1.1, with sin
    r3.v_y_ = speed_y;
    r3.orientation_ = orientation + 2.0*M_PI;
    r3.angular_speed_ = angular_speed;
    r3.has_orientation_ = true;
    r3.has_id_ = true;
    
    RobotDetection r4 = RobotDetection();
    r4.x_ = offset_x - 100.0; //will variate btw 0.9 and 1.1, with cos
    r4.y_ = offset_y + 100.0; //will variate btw 0.8 and 1.2, with sin + pi/4
    r4.v_x_ = speed_x; //will variate btw 0.8 and 1.2, with cos
    r4.v_y_ = speed_y;
    r4.orientation_ = orientation + 0.3;
    r4.angular_speed_ = angular_speed;
    r4.has_orientation_ = true;
    r4.has_id_ = true;


    robot_views[0] = &r2;

    robot_views[1] = &r3;

    robot_views[3] = &r4;

    for(int k=0; k<50; ++k){

        printf("New execution starts:\nNow calling Kalman filter... for the %d execution...\n", k);
        printf("Angular speed is 1 rad/s\n");
        printf("Previous position was : ");
        std::cout << t.position_;
        printf("\n");
        printf("Kalman execution ongoing...\n");
        t = f.kalmanFilter(robot_views, cadence_time);

        printf("New position is : ");
        std::cout << t.position_;
        printf(" while true position is ");
        std::cout << true_position;
        printf("and average algorithm yields ");
        //start of barycenter computing
        average_position = computeAverage(robot_views);
        //end of barycenter computing
        std::cout << average_position;
        printf("\n");

        printf("Update of robot_views and cadence time...");
        //update of robot_views
        for(unsigned int i = 0; i<ai::Config::NB_CAMERAS; ++i){
            if(robot_views[i]!= nullptr){
                if(cadence_time != 10) {
                    robot_views[i]->x_ = robot_views[i]->x_ + speed_x;
                    robot_views[i]->y_ = robot_views[i]->y_ + speed_y;
                    robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);

                    if(i == 0) {
                        robot_views[i]->v_x_ = speed_x + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                        robot_views[i]->x_ = robot_views[i]->x_+ 0.1*cos(cadence_time)*robot_views[i]->x_; 
                        robot_views[i]->y_ = robot_views[i]->y_ + 0.1*cos(cadence_time + M_PI/4)*robot_views[i]->y_; 
                    }
                    if(i == 2){
                        robot_views[i]->v_x_ = speed_x + 0.1*sin(cadence_time)*robot_views[i]->v_x_; 
                        robot_views[i]->x_ = robot_views[i]->x_ + 0.1*cos(cadence_time)*robot_views[i]->x_; 
                        robot_views[i]->y_ = robot_views[i]->y_ + 0.1*sin(cadence_time + M_PI/4)*robot_views[i]->y_; 
                    }
                    if(i == 3) {
                        robot_views[i]->v_x_ = speed_y + 0.2*cos(cadence_time)*robot_views[i]->v_x_; 
                        robot_views[i]->y_ = robot_views[i]->y_ + 0.2*sin(cadence_time + M_PI/4)*robot_views[i]->y_; 
                        robot_views[i]->x_ = robot_views[i]->x_ + 0.1*cos(cadence_time)*robot_views[i]->x_; 
                    }
                    if(i == 4){
                        robot_views[i]->v_x_ = speed_y + 0.1*cos(cadence_time)*robot_views[i]->v_x_; 
                        robot_views[i]->y_ = robot_views[i]->y_ + 0.1*cos(cadence_time + M_PI/4)*robot_views[i]->y_; 
                        robot_views[i]->x_ = robot_views[i]->x_ + 0.2*sin(cadence_time+M_PI/4)*robot_views[i]->x_; 

                    }

                    
                }

                else if (cadence_time == 10) { //Robot teleports
                    robot_views[i]->x_ = offset_x2;
                    robot_views[i]->y_ = offset_y2;
                    robot_views[i]->orientation_ = std::fmod(robot_views[i]->orientation_ + angular_speed, 2.0*M_PI);
                    
                }
            }
        }
        //true position update
        if(cadence_time != 10)
            true_position.linear += rhoban_geometry::Point((cadence_time - previous_time)*speed_x/1000, (cadence_time - previous_time)*speed_y/1000);
        else {
            true_position.linear = rhoban_geometry::Point(offset_x2/1000, offset_y2/1000);
        }
        true_position.angular = ContinuousAngle(std::fmod(cadence_time,2.0*M_PI));
        //time update
        previous_time = cadence_time;
        cadence_time += 1.0;
        printf("done...\n\n\n");
    }
}



int main() {

    //testKalman();
    //testKalmanNullptr();
    //testKalmanNoId();
    testKalman2();
    //testKalmanRobotTeleport();
    return EXIT_SUCCESS;
}