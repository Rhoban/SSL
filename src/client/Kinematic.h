#pragma once

#include <rhoban_geometry/point.h>

namespace rhoban_ssl
{
class Kinematic
{
public:
  struct WheelsSpeed
  {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
  };

  Kinematic();

  /**
   * Computes the kinematics for the robot
   * X and Y speed are [m/s], theta speed is [rad/s]
   * It returns the wheel speed in [turn/s]
   */
  WheelsSpeed compute(double xSpeed, double ySpeed, double thetaSpeed);

protected:
  // Angle between the -x axis and, respectively the rear wheel and the front
  // wheels [rad]
  double angleRear;
  double angleFront;

  // Radius of one wheel [m]
  double wheelRadius;

  // Radius of the robot [m]
  double robotRadius;

  // Wheel vectors
  rhoban_geometry::Point frontLeftVector;
  rhoban_geometry::Point frontRightVector;
  rhoban_geometry::Point backLeftVector;
  rhoban_geometry::Point backRightVector;

  rhoban_geometry::Point makeWheelVector(double angle);

  double computeFor(rhoban_geometry::Point vector, double xSpeed, double ySpeed, double thetaSpeed);
};
}  // namespace rhoban_ssl
