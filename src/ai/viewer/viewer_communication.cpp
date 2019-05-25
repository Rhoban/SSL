#include "viewer_communication.h"
#include "viewer/viewer_data_global.h"

namespace rhoban_ssl
{
namespace viewer
{
ViewerCommunication::ViewerCommunication(AI* ai)
  : ai_(ai), last_sending_time_(rhoban_utils::TimeStamp::now().getTimeMS())
{
}

bool ViewerCommunication::runTask()
{
  processIncomingPackets();

  if (GlobalDataSingleThread::singleton_.ai_data_.time - last_sending_time_ > SENDING_DELAY)
  {
    sendStatusPackets();
    last_sending_time_ = GlobalDataSingleThread::singleton_.ai_data_.time;
  }

  return true;
}

void ViewerCommunication::processIncomingPackets()
{
  while (!viewer::ViewerDataGlobal::get().received_packets.empty())
  {
    Json::Value viewer_packet = viewer::ViewerDataGlobal::get().received_packets.front();
    if (viewer_packet["action"] != "")
    {
      if (viewer_packet["action"] == "emergency")
      {
        ai_->api.emergency();
      }
      else
      {
        DEBUG("Aucune action trouvé");
      }
    }
    viewer::ViewerDataGlobal::get().received_packets.pop();
  }
}

void ViewerCommunication::sendStatusPackets()
{
  // GlobalData status
  viewer::ViewerDataGlobal::get().addPacket(fieldStatus());
  viewer::ViewerDataGlobal::get().addPacket(mobilesStatus());

  // Information of the ai
  viewer::ViewerDataGlobal::get().addPacket(availableManager());
  viewer::ViewerDataGlobal::get().addPacket(availableRobotBehavior());
}

Json::Value ViewerCommunication::fieldStatus()
{
  Json::Value packet;
  data::Field field = GlobalDataSingleThread::singleton_.field_;

  packet["field"]["field_width"] = field.field_width_;
  packet["field"]["field_length"] = field.field_length_;
  packet["field"]["boundary_width"] = field.boundary_width_;
  packet["field"]["goal_width"] = field.goal_width_;
  packet["field"]["goal_depth"] = field.goal_depth_;
  packet["field"]["penalty_area_width"] = field.penalty_area_width_;
  packet["field"]["penalty_area_depth"] = field.penalty_area_depth_;
  packet["field"]["circle"]["radius"] = field.circle_center_.getRadius();
  packet["field"]["circle"]["x"] = field.circle_center_.getCenter().getX();
  packet["field"]["circle"]["y"] = field.circle_center_.getCenter().getY();

  packet["informations"]["simulation"] = ai::Config::is_in_simulation;
  packet["informatons"]["color_ally"] = ai::Config::we_are_blue;

  return packet;
}

Json::Value ViewerCommunication::mobilesStatus()
{
  Json::Value packet;
  double time = GlobalDataSingleThread::singleton_.ai_data_.time;

  // Ball packet
  const rhoban_geometry::Point& ball_position =
      GlobalDataSingleThread::singleton_.ball_.getMovement().linearPosition(time);
  packet["ball"]["x"] = ball_position.getX();
  packet["ball"]["y"] = ball_position.getY();

  // Robot packet
  // #dbdd56 : Yellow color && #2393c6 : Blue color
  std::string color_ally = ai::Config::we_are_blue ? "#2393c6" : "#dbdd56";
  std::string color_opponent = ai::Config::we_are_blue ? "#dbdd56" : "#2393c6";

  for (int team = 0; team < 2; team++)
  {
    std::string team_side = team == 0 ? "ally" : "opponent";
    std::string team_color = team == 0 ? color_ally : color_opponent;

    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
    {
      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team][rid];
      const rhoban_geometry::Point& robot_position = current_robot.getMovement().linearPosition(time);

      packet[team_side][rid]["x"] = robot_position.getX();
      packet[team_side][rid]["y"] = robot_position.getY();
      packet[team_side][rid]["orientation"] = current_robot.getMovement().angularPosition(time).value();
      packet[team_side][rid]["is_present"] = current_robot.isActive();
      packet[team_side][rid]["id"] = current_robot.id;

      if (!ai::Config::is_in_simulation)
      {
        // Activate electronics.
        packet[team_side][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
        packet[team_side][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
      }
    }
  }
  return packet;
}

Json::Value ViewerCommunication::availableManager()
{
}

Json::Value ViewerCommunication::availableRobotBehavior()
{
}

}  // namespace viewer
}  // namespace rhoban_ssl
