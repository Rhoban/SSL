#include "quentin/CubicSpline.hpp"
#include "polyInterp.hpp"

int main(int argc, char* argv[]){
  
  PolyInterp interp;
  
  interp.addPoint(0, 0, 0, 0);
  interp.addPoint(0.5, 1, 1, 0);
  interp.addPoint(1, 0, 0, -1);
  interp.addPoint(0.5, -0.5, 0, 3);
  interp.addPoint(0.5, 0.5, 0, 0);

  Leph::Plot plot;

  for(double t = 0 ; t < interp.getCurrentTime() ; t+=0.01)
    plot.add({"time", t, "X", interp.getPoint(t)[0], "Y", interp.getPoint(t)[1]});
  
  plot.plot("X", "Y", Leph::Plot::Lines).render();
  
  return 0;
}
