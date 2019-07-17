#pragma once

#include <execution_manager.h>
#include <core/gnu_multi_plot.h>

#include <logger.h>

namespace rhoban_ssl
{
class PlotRobotReplay : public Task
{
  GnuMultiPlot plot;
  int rid;
  int nb_frames_;
  int current_frame;
  LogReplayTask* replayer_;

public:
  virtual ~PlotRobotReplay();
  PlotRobotReplay(int rid, int nb_frames, LogReplayTask* replayer);

  virtual bool runTask() override;
};
}  // namespace rhoban_ssl
