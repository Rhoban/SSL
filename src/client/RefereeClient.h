#pragma once

#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>
#include <ssl_referee.pb.h>
#include "MulticastClient.h"

namespace rhoban_ssl
{
class RefereeClient : public MulticastClient
{
public:
  RefereeClient();
  RefereeClient(std::string addr, std::string port);
  bool process(char* buffer, size_t len);
  Referee getData();

protected:
  Referee data;
};
}  // namespace rhoban_ssl
