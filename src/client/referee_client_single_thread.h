#pragma once

#include "multicast_client_single_thread.h"
#include "ssl_referee.pb.h"
#include <google/protobuf/arena.h>

namespace rhoban_ssl
{
class RefereeMessages
{
  google::protobuf::Arena* arena_;
  RefereeMessages();
  ~RefereeMessages();

public:
  Referee* getNewPacket();
  static RefereeMessages singleton_;
  std::list<Referee*> last_packets_;
  void reset();
};

class RefereeProtoBufReset : public Task
{
  int counter_;
  int freq_;

public:
  RefereeProtoBufReset(int freq = 100);
  virtual bool runTask() override;
};

class RefereeClientSingleThread : public MulticastClientSingleThread
{
public:
  RefereeClientSingleThread(std::string addr, std::string port);
  virtual bool process(char* buffer_, size_t len) override;
};
}
