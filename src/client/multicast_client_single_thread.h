#pragma once

#include <vector>
#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>
#include <poll.h>

#include "execution_manager.h"

namespace rhobanssl
{
/**
 * This is a generic multicast client that listens on all possible interfaces
 * (running one thread per interface) for incoming packets
 */
class MulticastClientSingleThread : public virtual rhobanssl::Task
{
public:
  struct Interface
  {
    std::string name;
    int family;
    unsigned int index;
  };

  /**
 * At instanciation, provide the address and the port to listen to for the
 * multicast client to listen to
 *
 * @param addr Multicast address to listen
 * @param port Multicast port to listen
 */
  MulticastClientSingleThread(std::string addr, std::string port);
  virtual ~MulticastClientSingleThread();

  /**
 * Process an incoming packet, must be implemented in subclass and will be called to process data in buffer
 *
 * @param  buffer  A buffer containing received bytes
 * @param  len     Number of bytes in the buffer
 * @return         Returns true if the contents is a valid packet
 */
  virtual bool process(char* buffer, size_t len) = 0;

  /**
 * Is there valid received packet ?
 */
  bool hasData() const;

  /**
 * How many packets were received ?
 */
  unsigned int getPackets();

  void shutdown();

  bool runTask();

protected:
  struct pollfd* sockets_fds_;
  int nfds_;
  std::string addr, port;
  bool receivedData;
  volatile bool running;
  rhoban_utils::TimeStamp lastData;
  unsigned int packets;

  /**
 * Initializes the multicast client
 */
  void init();

  /**
 * Called when a valid packet incomes, can be overloaded in sub classes
 * to trigger events when a packet is received
 */
  virtual void packetReceived();
};

}  // namespace RhobanSSL