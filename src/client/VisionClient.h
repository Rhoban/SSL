#pragma once

#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>
#include <messages_robocup_ssl_wrapper.pb.h>
#include "MulticastClient.h"
#include "client_config.h"

namespace rhoban_ssl
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
  SSL_WrapperPacket data;
};
}  // namespace rhoban_ssl
