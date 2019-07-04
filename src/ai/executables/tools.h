#pragma once

#include <string>
#include <vision/robot_position_filter.h>

namespace rhoban_ssl
{
namespace ai
{
class AI;
}

void addCoreTasks();
void addVisionTasks(std::string vision_addr, std::string vision_port, vision::PartOfTheField part_of_the_field_used);
void addRefereeTasks();
void addViewerTasks(ai::AI* ai, int port);
void addPreBehaviorTreatment();
void addRobotComTasks();
void addTaskShortCutProcessIfNoVisionData();
}
