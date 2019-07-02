#pragma once

#include <execution_manager.h>
#include <core/gnu_plot.h>

namespace rhoban_ssl
{
class PlotVelocity : public Task
{
  GnuPlot plot;
  int rid;
  std::chrono::high_resolution_clock::time_point start;

public:
  ~PlotVelocity();
  PlotVelocity(int rid);

  bool runTask();
};
}
