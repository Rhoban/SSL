#include "quentin/CubicSpline.hpp"
#include <Eigen/Dense>

class PolyInterp{
  
public:
  PolyInterp();
  Eigen::Vector2d getPoint(double t); 
  void addPoint(double x, double y, double velocityX, double velocityY);
  void addPoint(Eigen::Vector2d point, Eigen::Vector2d velocity);
  double getCurrentTime();
  
protected:
  Leph::CubicSpline cubicSplineX;
  Leph::CubicSpline cubicSplineY;

  double currentTime;
};
