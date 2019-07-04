#include "tools.h"
#include <execution_manager.h>
#include <ai.h>
#include <referee_client_single_thread.h>
#include <referee/referee_packet_analyzer.h>
#include <data/computed_data.h>
#include <viewer/viewer_communication.h>
#include <viewer_server.h>
#include <vision/ai_vision_client.h>

namespace rhoban_ssl
{
void addCoreTasks()
{
  ExecutionManager::getManager().addTask(new ai::InitMobiles(), 0);
  ExecutionManager::getManager().addTask(new ai::TimeUpdater(), 299);
}

void addVisionTasks(std::string vision_addr, std::string vision_port, vision::PartOfTheField part_of_the_field_used)
{  // range 200
  ExecutionManager::getManager().addTask(new vision::VisionClientSingleThread(vision_addr, vision_port), 200);
  // ExecutionManager::getManager().addTask(new vision::VisionPacketStat(100));
  ExecutionManager::getManager().addTask(new vision::SslGeometryPacketAnalyzer(), 210);
  ExecutionManager::getManager().addTask(new vision::DetectionPacketAnalyzer(), 220);
  ExecutionManager::getManager().addTask(new vision::ChangeReferencePointOfView(), 230);
  ExecutionManager::getManager().addTask(new vision::UpdateRobotInformation(part_of_the_field_used), 240);
  ExecutionManager::getManager().addTask(new vision::UpdateBallInformation(part_of_the_field_used), 250);
  // ExecutionManager::getManager().addTask(new vision::VisionDataTerminalPrinter());
  ExecutionManager::getManager().addTask(new vision::VisionProtoBufReset(10), 10000);
  ExecutionManager::getManager().addTask(new ConditionalTask(
                                             []() -> bool {  // wait for at least 30 packets from vision
                                               static int counter = 0;
                                               counter += vision::VisionDataGlobal::singleton_.last_packets_.size();
                                               return counter > 30;
                                             },
                                             []() -> bool {
                                               DEBUG("we receive enought vision packet data to activate other tasks");
                                               ExecutionManager::getManager().setMaxTaskId();
                                               return false;
                                             }),
                                         299);
  ExecutionManager::getManager().setMaxTaskId(300);
}

void addRefereeTasks()
{  // range 100
  ExecutionManager::getManager().addTask(new referee::RefereeClientSingleThread(SSL_REFEREE_ADDRESS, SSL_REFEREE_PORT),
                                         100);
  ExecutionManager::getManager().addTask(new referee::RefereePacketAnalyzer(), 110);
  // ExecutionManager::getManager().addTask(new referee::RefereeTerminalPrinter());
  ExecutionManager::getManager().addTask(new referee::RefereeProtoBufReset(10), 120);
}

void addPreBehaviorTreatment()
{  // range 300
  ExecutionManager::getManager().addTask(new data::CollisionComputing(), 300);
}

void addRobotComTasks()
{  // range 2000
  ExecutionManager::getManager().addTask(new control::LimitVelocities(), 2000);
  ExecutionManager::getManager().addTask(new control::Commander(), 2010);
}

void addViewerTasks(ai::AI* ai, int port)
{  // range 3000
  ExecutionManager::getManager().addTask(new viewer::ViewerServer(port), 3000);
  ExecutionManager::getManager().addTask(new viewer::ViewerCommunication(ai), 3010);
}

class ShortCutVision : public Task
{
public:
  virtual bool runTask() override
  {
    if (vision::VisionDataGlobal::singleton_.last_packets_.size() > 0)
      ExecutionManager::getManager().setMaxTaskId();
    else
      ExecutionManager::getManager().setMaxTaskId(300);
  }
};

void addTaskShortCutProcessIfNoVisionData()
{
  ExecutionManager::getManager().addTask(new ShortCutVision(), 299);
  ExecutionManager::getManager().setMaxTaskId(300);
}
}
