#include <iostream>
#include <cmath>
#include "Kinematic.h"

int main()
{
  RhobanSSL::Kinematic kinematic;

  auto wheels = kinematic.compute(0, 1, 0);
  std::cout << "wheels: " << std::endl;
  std::cout << "- front left: " << wheels.frontLeft << std::endl;
  std::cout << "- front right: " << wheels.frontRight << std::endl;
  std::cout << "- back left: " << wheels.backLeft << std::endl;
  std::cout << "- back right: " << wheels.backRight << std::endl;
}
