#include "polyInterp.hpp"

PolyInterp::PolyInterp(){
  currentTime = 0.; 
}

void PolyInterp::addPoint(double x, double y, double velocityX, double velocityY){
  cubicSplineX.addPoint(currentTime, x, velocityX);
  cubicSplineY.addPoint(currentTime, y, velocityY);
  currentTime++;
}

void PolyInterp::addPoint(Eigen::Vector2d point, Eigen::Vector2d velocity){
  cubicSplineX.addPoint(currentTime, point[0], velocity[0]);
  cubicSplineY.addPoint(currentTime, point[1], velocity[1]);
  currentTime++;  
}

Eigen::Vector2d PolyInterp::getPoint(double t){
  t *= currentTime;
  return Eigen::Vector2d(cubicSplineX.pos(t), cubicSplineY.pos(t));
}

double PolyInterp::getCurrentTime(){
  return currentTime;
}
