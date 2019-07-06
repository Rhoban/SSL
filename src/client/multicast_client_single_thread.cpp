#include <string.h>
#include <unistd.h>
#include <chrono>
#include <map>
#include <ifaddrs.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <iostream>
#include "multicast_client_single_thread.h"

using namespace rhoban_utils;

namespace rhoban_ssl
{
MulticastClientSingleThread::MulticastClientSingleThread(std::string addr, std::string port)
  : sockets_fds_(nullptr), addr(addr), port(port), receivedData(false), packets(0)
{
}

void MulticastClientSingleThread::init()
{
  // Listing interfaces
  std::map<unsigned int, Interface> interfaces;
  ifaddrs* ifs = nullptr;

  char* ifname = getenv("SSL_MULTICAST_IFNAME");
  if (getifaddrs(&ifs) < 0)
  {
    std::cerr << "Can't get network interface list" << std::endl;
    exit(0);
  }
  try
  {
    for (const ifaddrs* i = ifs; i; i = i->ifa_next)
    {
      if ((i->ifa_flags & IFF_UP) && (i->ifa_flags & IFF_MULTICAST) && i->ifa_addr)
      {
        if (i->ifa_addr->sa_family == AF_INET /*|| i->ifa_addr->sa_family == AF_INET6*/)
        {
          if ((ifname == nullptr) || (strcmp(i->ifa_name, ifname) == 0))
          {
            unsigned int ifindex = if_nametoindex(i->ifa_name);
            if (ifindex)
            {
              Interface interface;
              interface.name = i->ifa_name;
              interface.family = i->ifa_addr->sa_family;
              interface.index = ifindex;
              interfaces[ifindex] = interface;
            }
          }
        }
      }
    }
    freeifaddrs(ifs);
  }
  catch (...)
  {
    freeifaddrs(ifs);
    throw;
  }

  // For each interface, running a thread
  running = true;

  if (interfaces.size() == 0)
  {
    std::cerr << "\033[31;5mWARNING:\033[0m there is no interface wih multicast support! " << std::endl;
    std::cerr << "         considere using ifconfig to check your setup " << std::endl;
  }
  if (interfaces.size() > 1)
  {
    std::cerr << "\033[31;5mWARNING:\033[0m there is more than one interface wih multicast support! " << std::endl;
    std::cerr << "         This will probably lead to duplicate packets reception " << std::endl;
    std::cerr << "         Check your 'interface' with ifconfig and turn multicast off " << std::endl;
    std::cerr << "         on unwanted iterface with command: ifconfig eth0 -muticast" << std::endl;
    std::cerr << "         or set environment variable SSL_MULTICAST_IFNAME to one of the following values"
              << std::endl;
    for (auto it = interfaces.begin(); it != interfaces.end(); ++it)
      std::cerr << "         export SSL_MULTICAST_IFNAME=" << it->second.name << std::endl;
  }

  sockets_fds_ = new struct pollfd[interfaces.size()];
  nfds_ = 0;

  for (auto entry : interfaces)
  {
    // auto interface = entry.second;
    //    std::thread* thread = new std::thread([this, interface]() { run(interface.family, interface.index); });

    //  threads.push_back(thread);

    // This is mainly from ssl-refbox/client example

    // Get a socket address to bind to.
    addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = 0;
    hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;
    addrinfo* ai = nullptr;
    int gai_err;
    if ((gai_err = getaddrinfo(nullptr, port.c_str(), &hints, &ai)) != 0)
    {
      std::cerr << gai_strerror(gai_err);
      continue;
    }

    // Create and bind a socket.
    int sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);

    int enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
      std::cerr << "Can't REUSEADDR" << '\n';
      continue;
    }

    if (sock < 0)
    {
      std::cerr << strerror(errno) << '\n';
      continue;
    }
    if (bind(sock, ai->ai_addr, ai->ai_addrlen) < 0)
    {
      std::cerr << strerror(errno) << '\n';
      continue;
    }

    freeaddrinfo(ai);

    // Join the multicast group.
    ip_mreqn mcreq;
    mcreq.imr_multiaddr.s_addr = inet_addr(addr.c_str());
    mcreq.imr_address.s_addr = INADDR_ANY;
    mcreq.imr_ifindex = entry.second.index;
    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mcreq, sizeof(mcreq)) < 0)
    {
      std::cerr << strerror(errno) << '\n';
      continue;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));

    sockets_fds_[nfds_].fd = sock;
    sockets_fds_[nfds_].events = POLLIN;
    nfds_ += 1;
  }
  if (nfds_ == 0)
  {
    std::cerr << "no interface to listen on (MulticastClientSingleThread)" << std::endl;
    _exit(1);
  }

  memset(msgs, 0, sizeof(msgs));
  for (int j = 0; j < VLEN; j++)
  {
    iovecs[j].iov_base = bufs[j];
    iovecs[j].iov_len = BUFSIZE;
    msgs[j].msg_hdr.msg_iov = &iovecs[j];
    msgs[j].msg_hdr.msg_iovlen = 1;
  }
}

MulticastClientSingleThread::~MulticastClientSingleThread()
{
  running = false;
  if (sockets_fds_ != nullptr)
    delete[] sockets_fds_;
  /*
for (auto thread : threads)
{
  thread->join();
  delete thread;
}
*/
}

//#define VLEN 20
//#define BUFSIZE 100

bool MulticastClientSingleThread::runTask()
{
  // This is mainly from ssl-refbox/client example

  if (running == false)
    return false;

  for (unsigned int i = 0; i < nfds_; ++i)
    sockets_fds_[i].revents = 0;

  poll(sockets_fds_, nfds_, 0);

  for (unsigned int i = 0; i < nfds_; ++i)
  {
    if (sockets_fds_[i].revents & POLLIN)
    {  // can be static as we are in single thread paradigm
       // ssize_t len = recv(sockets_fds_[i].fd, buffer_, sizeof(buffer_), 0);

      int len = recvmmsg(sockets_fds_[i].fd, msgs, VLEN, 0, nullptr);

      for (int k = 0; k < len; ++k)
      {
        if (process(bufs[k], msgs[k].msg_len))
        {
          packets++;
          // packetReceived();
          receivedData = true;
          lastData = TimeStamp::now();
        }
      }
    }
  }
  return true;
}

bool MulticastClientSingleThread::hasData() const
{
  if (receivedData)
  {
    auto delta = diffMs(lastData, TimeStamp::now());
    return delta < 3000;
  }

  return false;
}

// void MulticastClientSingleThread::packetReceived()
//{
// Default behaviors does nothing
//}

unsigned int MulticastClientSingleThread::getPackets()
{
  return packets;
}

void MulticastClientSingleThread::shutdown()
{
  running = false;
}
}  // namespace rhoban_ssl
