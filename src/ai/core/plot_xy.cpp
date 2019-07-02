#include "plot_xy.h"
#include <data.h>

rhoban_ssl::PlotXy::~PlotXy()
{
  plot.closeWindow();
}

rhoban_ssl::PlotXy::PlotXy(int rid) : rid(rid), start(std::chrono::high_resolution_clock::now())
{
}

bool rhoban_ssl::PlotXy::runTask()
{
  plot.setX(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
  // plot.push("x", Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.norm());
  plot.push("x", Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->ai_data.time).x);
  plot.push("y", Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->ai_data.time).y);
  plot.push("velocity x", Data::get()->robots[Ally][rid].getMovement().linearVelocity(Data::get()->ai_data.time)[0]);
  plot.push("velocity y", Data::get()->robots[Ally][rid].getMovement().linearVelocity(Data::get()->ai_data.time)[1]);
  plot.render();
  return true;
}
