#pragma once

#include <execution_manager.h>
#include <core/gnu_multi_plot.h>

namespace rhoban_ssl
{
class PlotRobot : public Task
{
  GnuMultiPlot plot;
  int rid;

public:
  ~PlotRobot();
  PlotRobot(int rid);

  bool runTask();
};
}  // namespace rhoban_ssl
