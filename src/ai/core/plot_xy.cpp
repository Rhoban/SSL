#include "plot_xy.h"
#include <data.h>

rhoban_ssl::PlotXy::~PlotXy()
{
  plot.closeWindow();
}

rhoban_ssl::PlotXy::PlotXy(int rid) : rid(rid)
{
}

bool rhoban_ssl::PlotXy::runTask()
{
  plot.setX(Data::get()->robots[Ally][rid].movement_sample[0].time);
  // plot.push("x", Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.norm());
  plot.push("x", Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).x);
  plot.push("y", Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).y);
  plot.push("velocity x", Data::get()->robots[Ally][rid].getMovement().linearVelocity(Data::get()->time.now())[0]);
  plot.push("velocity y", Data::get()->robots[Ally][rid].getMovement().linearVelocity(Data::get()->time.now())[1]);
  plot.render();
  return true;
}
