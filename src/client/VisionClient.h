#pragma once

#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>
#include <messages_robocup_ssl_wrapper.pb.h>
#include "MulticastClient.h"
#include "multicast_client_single_thread.h"
#include "client_config.h"
#include <google/protobuf/arena.h>

namespace rhoban_ssl
{
namespace vision
{
class VisionClient : public MulticastClient
{
public:
  // VisionClient(bool simulation = false);
  VisionClient(bool simulation = false, std::string addr = SSL_VISION_ADDRESS, std::string port = SSL_VISION_PORT,
               std::string sim_port = SSL_SIMULATION_VISION_PORT);
  bool process(char* buffer, size_t len);
  SSL_WrapperPacket getData();

protected:
  SSL_WrapperPacket data_;
};

class VisionDataGlobal
{
  VisionDataGlobal();
  ~VisionDataGlobal();

  google::protobuf::Arena* arena_;

public:
  // std::list<SSL_WrapperPacket*> packets_buffer_;
  std::list<SSL_WrapperPacket*> last_packets_;
  static VisionDataGlobal singleton_;
  SSL_WrapperPacket* getNewPacket();
  void reset();
};

class VisionProtoBufReset : public Task
{
  int counter_;
  int freq_;

public:
  VisionProtoBufReset(int freq = 100);
  virtual bool runTask() override;
};

class VisionClientSingleThread : public MulticastClientSingleThread
{
public:
  VisionClientSingleThread(std::string addr, std::string port);
  virtual bool process(char* buffer_, size_t len) override;
};
}  // namespace vision
}  // namespace rhoban_ssl
