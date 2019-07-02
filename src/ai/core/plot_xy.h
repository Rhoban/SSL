#pragma once

#include <execution_manager.h>
#include <core/gnu_plot.h>

namespace rhoban_ssl
{
class PlotXy : public Task
{
  GnuPlot plot;
  int rid;
  std::chrono::high_resolution_clock::time_point start;

public:
  ~PlotXy();
  PlotXy(int rid);

  bool runTask();
};
}
