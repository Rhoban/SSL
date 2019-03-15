#include <chrono>

#include <iostream>
#include "VisionClient.h"
#include "client_config.h"

using namespace rhoban_utils;

namespace RhobanSSL
{
// VisionClient::VisionClient(bool simulation) :
//   MulticastClient(SSL_VISION_ADDRESS, SSL_VISION_PORT)
// {
//   if (simulation) {
//     port = SSL_SIMULATION_VISION_PORT;
//   }

//   init();
// }

VisionClient::VisionClient(bool simulation, std::string addr, std::string port, std::string sim_port)
  : MulticastClient(addr, port)
{
  if (simulation)
  {
    port = sim_port;
  }
  std::cout << "Vision client (simulation=" << ((simulation) ? "True" : "False") << "): " << addr << ":" << port
            << std::endl;

  init();
}

SSL_WrapperPacket VisionClient::getData()
{
  SSL_WrapperPacket tmp;

  mutex.lock();
  tmp = data;
  mutex.unlock();

  return tmp;
}

bool VisionClient::process(char* buffer, size_t len)
{
  SSL_WrapperPacket packet;

  if (packet.ParseFromArray(buffer, len))
  {
    data = packet;

    return true;
  }
  else
  {
    std::cerr << "Packet error!" << std::endl;
  }

  return false;
}
}  // namespace RhobanSSL
