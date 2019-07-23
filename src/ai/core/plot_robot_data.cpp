#include "plot_robot_data.h"
#include <data.h>

rhoban_ssl::PlotRobot::~PlotRobot()
{
  plot.closeWindow();
}

rhoban_ssl::PlotRobot::PlotRobot(int rid) : rid(rid)
{
  plot.setDisplayedValues(50);
}

bool rhoban_ssl::PlotRobot::runTask()
{
  plot.add("position", "x", Data::get()->robots[Ally][rid].movement_sample[0].time,
           Data::get()->robots[Ally][rid].movement_sample[0].linear_position.x);
  plot.add("position", "y", Data::get()->robots[Ally][rid].movement_sample[0].time,
           Data::get()->robots[Ally][rid].movement_sample[0].linear_position.y);
  plot.add("position", "predX", Data::get()->time.now(),
           Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).x);
  plot.add("position", "predY", Data::get()->time.now(),
           Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now()).y);
  /* plot.add("position", "predX+0.01", Data::get()->time.now() + 0.01,
            Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now() + 0.01).x);
   plot.add("position", "predY+0.01", Data::get()->time.now() + 0.01,
            Data::get()->robots[Ally][rid].getMovement().linearPosition(Data::get()->time.now() + 0.01).y);
            */
  plot.add("angular", "angle", Data::get()->robots[Ally][rid].movement_sample[0].time,

           Data::get()->robots[Ally][rid].movement_sample[0].angular_position.value());
  plot.add("angular", "pred angle", Data::get()->time.now(),
           Data::get()->robots[Ally][rid].getMovement().angularPosition(Data::get()->time.now()).value());
  plot.add("control", "x", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.getX());
  plot.add("control", "y", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.getY());
  plot.add("control norm", "norm", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.linear_velocity.norm());
  /*
  plot.add("control order", "kick", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.kick ? 1 : 0);
  plot.add("control order", "spin", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.spin ? 1 : 0);
  plot.add("control order", "charge", Data::get()->time.now(),
           Data::get()->shared_data.final_control_for_robots[rid].control.charge ? 1 : 0);
           */
  plot.render();
  return true;
}
