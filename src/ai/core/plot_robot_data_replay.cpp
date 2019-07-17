#include "plot_robot_data_replay.h"
#include <data.h>

namespace rhoban_ssl
{
PlotRobotReplay::~PlotRobotReplay()
{
  plot.closeWindow();
}

PlotRobotReplay::PlotRobotReplay(int rid, int nb_frames, LogReplayTask* replayer)
  : rid(rid), nb_frames_(nb_frames), replayer_(replayer)
{
  plot.setDisplayedValues(nb_frames_);
}

bool PlotRobotReplay::runTask(void)
{
  if (replayer_->currentFrame() != current_frame)
  {
    current_frame = replayer_->currentFrame();
    plot.clear();
    for (int fid = current_frame - nb_frames_; fid <= current_frame; fid++)
    {
      plot.add("position", "x", replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].time,
               replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].linear_position.x);
      plot.add("position", "y", replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].time,
               replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].linear_position.y);
      //      plot.add("position", "predX", replayer_->getFrame(fid)->time.now(),
      //               replayer_->getFrame(fid)->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).x);
      //      plot.add("position", "predY", replayer_->getFrame(fid)->time.now(),
      //               replayer_->getFrame(fid)->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).y);
      plot.add("angular", "angle", replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].time,
               replayer_->getFrame(fid)->robots[Ally][rid].movement_sample[0].angular_position.value());
      //      plot.add("angular", "pred angle", replayer_->getFrame(fid)->time.now(),
      //               replayer_->getFrame(fid)->robots[Ally][rid].getMovement().angularPosition(Data::get()->time.now()).value());
      plot.add("control", "x", replayer_->getFrame(fid)->time.start_loop_time,
               replayer_->getFrame(fid)->shared_data.final_control_for_robots[rid].control.linear_velocity.getX());
      plot.add("control", "y", replayer_->getFrame(fid)->time.start_loop_time,
               replayer_->getFrame(fid)->shared_data.final_control_for_robots[rid].control.linear_velocity.getY());
      plot.add("control norm", "norm", Data::get()->time.start_loop_time,
               Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.norm());
      plot.add("control order", "kick", Data::get()->time.start_loop_time,
               Data::get()->shared_data.final_control_for_robots[rid].control.kick ? 1 : 0);
      plot.add("control order", "spin", Data::get()->time.start_loop_time,
               Data::get()->shared_data.final_control_for_robots[rid].control.spin ? 1 : 0);
      plot.add("control order", "charge", Data::get()->time.start_loop_time,
               Data::get()->shared_data.final_control_for_robots[rid].control.charge ? 1 : 0);
    }
    plot.render();
  }
  return true;
}
}
