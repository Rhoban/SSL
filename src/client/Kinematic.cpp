#include <geometry/Angle.hpp>
#include <cmath>
#include "Kinematic.h"

namespace RhobanSSL
{
    Kinematic::Kinematic()
    {
        angleRear = deg2rad(45);
        angleFront = deg2rad(120);
        wheelRadius = 0.05;     // XXX: To measure
        robotRadius = 0.175;    // XXX: To measure

        frontLeftVector = makeWheelVector(angleFront);
        frontRightVector = makeWheelVector(-angleFront);
        backLeftVector = makeWheelVector(angleRear);
        backRightVector = makeWheelVector(-angleRear);
    }

    Kinematic::WheelsSpeed Kinematic::compute(double xSpeed, double ySpeed, double thetaSpeed)
    {
        WheelsSpeed wheels;

        wheels.frontLeft = computeFor(frontLeftVector, xSpeed, ySpeed, thetaSpeed);
        wheels.frontRight = computeFor(frontRightVector, xSpeed, ySpeed, thetaSpeed);
        wheels.backLeft = computeFor(backLeftVector, xSpeed, ySpeed, thetaSpeed);
        wheels.backRight = computeFor(backRightVector, xSpeed, ySpeed, thetaSpeed);

        return wheels;
    }

    double Kinematic::computeFor(Point vector, double xSpeed, double ySpeed, double thetaSpeed)
    {
        // Speed at wheel for translation [m/s]
        double speedAtWheel = Point::dotProduct(vector, Point(xSpeed, ySpeed));

        // Adding rortation to speed at wheel
        speedAtWheel += thetaSpeed * robotRadius;

        // Converting speed at wheel in [turn/s]
        double speed = speedAtWheel / (2*M_PI*wheelRadius);

        return speed;
    }

    Point Kinematic::makeWheelVector(double angle)
    {
        Point vector(0, -1);

        vector = vector.rotation(Angle(rad2deg(angle)));

        return vector;
    }
}
