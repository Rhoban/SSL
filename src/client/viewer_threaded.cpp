#include "viewer_server.h"
#include <google/protobuf/stubs/logging.h>

static volatile bool running = true;
void stop(int s)
{
  running = false;
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

struct Field
{
  double field_length_ = 9.0;
  double field_width_ = 6.0;
  double goal_width_ = 1.0;
  double goal_depth_ = 0.02;
  double boundary_width_ = 0.02;
  double penalty_area_depth_ = 0.5;
  double penalty_area_width_ = 3.0;

  double circle_x = 0;
  double circle_y = 0;
  double circle_radius = 0.3;
};

class SendFieldPacket : public rhoban_ssl::Task
{
public:
  virtual bool runTask() override
  {
    if (rhoban_ssl::viewer::ViewerDataGlobal::get().client_connected)
    {
      Field field;
      Json::Value packet;

      packet["field"]["length"] = field.field_length_;
      packet["field"]["width"] = field.field_width_;
      packet["field"]["boundary_width"] = field.boundary_width_;
      packet["field"]["goal"]["width"] = field.goal_width_;
      packet["field"]["goal"]["depth"] = field.goal_depth_;
      packet["field"]["penalty_area"]["width"] = field.penalty_area_width_;
      packet["field"]["penalty_area"]["depth"] = field.penalty_area_depth_;
      packet["field"]["circle"]["x"] = field.circle_x;
      packet["field"]["circle"]["y"] = field.circle_y;
      packet["field"]["circle"]["radius"] = field.circle_radius;

      rhoban_ssl::viewer::ViewerDataGlobal::get().packets_to_send.push(packet);
    }
    return running;
  }
};

int main()
{
  signal(SIGINT, stop);

  rhoban_ssl::ExecutionManager::getManager().addTask(new rhoban_ssl::viewer::ViewerServer(7882));
  rhoban_ssl::ExecutionManager::getManager().addTask(new SendFieldPacket());
  rhoban_ssl::ExecutionManager::getManager().run(0.5);
  return 0;
}
