#include <chrono>

#include <iostream>
#include "RefereeClient.h"
#include "client_config.h"
//#include <google/protobuf/util/json_util.h>

using namespace rhoban_utils;

namespace rhoban_ssl
{
RefereeClient::RefereeClient() : MulticastClient(SSL_REFEREE_ADDRESS, SSL_REFEREE_PORT)
{
  init();
}

RefereeClient::RefereeClient(std::string addr, std::string port) : MulticastClient(addr, port)
{
  init();
}

SSL_Referee RefereeClient::getData()
{
  SSL_Referee tmp;

  mutex.lock();
  tmp = data;
  mutex.unlock();

  return tmp;
}

bool RefereeClient::process(char* buffer, size_t len)
{
  SSL_Referee packet;

  if (packet.ParseFromArray(buffer, len))
  {
    data = packet;
    return true;
  }

  return false;
}
}  // namespace rhoban_ssl
