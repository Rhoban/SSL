#include <signal.h>
#include "VisionClient.h"
#include <iostream>
#include <google/protobuf/stubs/logging.h>

static volatile bool running = true;
void stop(int s)
{
  running = false;
  rhoban_ssl::ExecutionManager::getManager().shutdown();
}

class DoubleFrameCleaner : public rhoban_ssl::Task
{
  int current_frame[4];

public:
  DoubleFrameCleaner()
  {
    for (int i = 0; i < 4; ++i)
      current_frame[i] = 0;
  }
  virtual bool runTask() override
  {
    std::list<SSL_WrapperPacket*> to_remove;
    for (auto i : rhoban_ssl::VisionDataGlobal::singleton_.last_packets_)
      if (i->has_detection())
      {
        if (current_frame[i->detection().camera_id()] == i->detection().frame_number())
          to_remove.push_back(i);
        if (current_frame[i->detection().camera_id()] < i->detection().frame_number())
          current_frame[i->detection().camera_id()] = i->detection().frame_number();
      }
    for (auto i : rhoban_ssl::VisionDataGlobal::singleton_.last_packets_)
      if ((current_frame[i->detection().camera_id()] > i->detection().frame_number()))
        to_remove.push_back(i);
    for (auto i : to_remove)
    {
      rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.remove(i);
      // rhoban_ssl::VisionDataGlobal::singleton_.packets_buffer_.push_back(i);
    }
    return running;
  }
};

class ProcessSSLPacket : public rhoban_ssl::Task
{
public:
  virtual bool runTask() override
  {
    while (rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.empty() == false)
    {
      SSL_WrapperPacket* p = rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.front();
      rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.pop_front();
      if (p->has_geometry())
      {
        std::cout << "receive a geometry packet" << std::endl;
      }
      if (p->has_detection())
      {
        std::cout << "receive a detection packet : " << p->detection().frame_number() << " for camera "
                  << p->detection().camera_id() << std::endl;
      }
      // rhoban_ssl::VisionDataGlobal::singleton_.packets_buffer_.push_back(p);
      //     delete p;
    }
    return running;
  }
};

int main()
{
  signal(SIGINT, stop);

  rhoban_ssl::ExecutionManager::getManager().addTask(
      new rhoban_ssl::VisionClientSingleThread(SSL_VISION_ADDRESS, SSL_SIMULATION_VISION_PORT));

  // rhoban_ssl::ExecutionManager::getManager().addTask(new DoubleFrameCleaner());
  rhoban_ssl::ExecutionManager::getManager().addTask(new ProcessSSLPacket());
  rhoban_ssl::ExecutionManager::getManager().addTask(new rhoban_ssl::VisionProtoBufReset(10));
  rhoban_ssl::ExecutionManager::getManager().run(0.01);
  ::google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
