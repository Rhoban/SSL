#include "plot_velocity.h"
#include <data.h>

rhoban_ssl::PlotVelocity::PlotVelocity(int rid) : rid(rid), start(std::chrono::high_resolution_clock::now())
{
}

bool rhoban_ssl::PlotVelocity::runTask()
{
  plot.setX(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
  plot.push("velocity", Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.norm());
  plot.render();
  return true;
}
