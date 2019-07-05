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

#include <string.h>
#include <fcntl.h>

using namespace rhoban_utils;

namespace rhoban_ssl
{
MulticastClientSingleThreadLegacy::MulticastClientSingleThreadLegacy(std::string addr, std::string port)
  : sockets_fds_(nullptr), addr(addr), port(port), receivedData(false), packets(0)
{
}

void MulticastClientSingleThreadLegacy::init()
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
    std::cerr << "         on unwanted interface with command:" << std::endl << "ifconfig eth0 -multicast" << std::endl;
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

MulticastClientSingleThreadLegacy::~MulticastClientSingleThreadLegacy()
{
  running = false;
  if (sockets_fds_ != nullptr)
    delete[] sockets_fds_;
}

bool MulticastClientSingleThreadLegacy::runTask()
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

}  // namespace rhoban_ssl

// Code from SSL-vision

namespace Net
{
//====================================================================//
//  Net::Address: Network address class
//  (C) James Bruce
//====================================================================//

template <class data>
inline void mzero(data& d)
{
  memset(&d, 0, sizeof(d));
}

class Address
{
  sockaddr addr;
  socklen_t addr_len;

public:
  Address()
  {
    memset(&addr, 0, sizeof(addr));
    addr_len = 0;
  }
  Address(const Address& src)
  {
    copy(src);
  }
  ~Address()
  {
    reset();
  }

  bool setHost(const char* hostname, int port)
  {
    // printf("%s %d\n",hostname,port);
    addrinfo* res = NULL;
    getaddrinfo(hostname, NULL, NULL, &res);
    if (!res)
      return (false);

    mzero(addr);
    addr_len = res->ai_addrlen;
    memcpy(&addr, res->ai_addr, addr_len);

    // set port for internet sockets
    sockaddr_in* sockname = (sockaddr_in*)(&addr);
    if (sockname->sin_family == AF_INET)
    {
      sockname->sin_port = htons(port);
    }
    else
    {
      // TODO: any way to set port in general?
    }

    freeaddrinfo(res);
    return (true);
  }
  void setAny(int port = 0)
  {
    mzero(addr);
    sockaddr_in* s = (sockaddr_in*)(&addr);
    s->sin_addr.s_addr = htonl(INADDR_ANY);
    s->sin_port = htons(port);
    addr_len = sizeof(sockaddr_in);
  }

  bool operator==(const Address& a) const
  {
    return (addr_len == a.addr_len && memcmp(&addr, &a.addr, addr_len) == 0);
  }
  void copy(const Address& src)
  {
    memcpy(&addr, &src.addr, src.addr_len);
    addr_len = src.addr_len;
  }
  void reset()
  {
    memset(&addr, 0, sizeof(addr));
    addr_len = 0;
  }
  void clear()
  {
    reset();
  }

  in_addr_t getInAddr() const
  {
    const sockaddr_in* s = (sockaddr_in*)(&addr);
    return (s->sin_addr.s_addr);
  }

  void print(FILE* out = stdout) const
  {
    if (!addr_len)
    {
      printf("null");
      return;
    }

    sockaddr_in* sockname = (sockaddr_in*)(&addr);
    if (sockname->sin_family == AF_INET)
    {
      unsigned a = ntohl(sockname->sin_addr.s_addr);
      unsigned short p = ntohs(sockname->sin_port);

      fprintf(out, "%d.%d.%d.%d:%d", (a >> 24) & 0xFF, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF, p);
    }
    else
    {
      fprintf(out, "?");
    }
  }

  friend class UDP;
};

//====================================================================//
//  Net::UDP: Simple raw UDP messaging
//  (C) James Bruce
//====================================================================//

class UDP
{
  int fd;

public:
  unsigned sent_packets;
  unsigned sent_bytes;
  unsigned recv_packets;
  unsigned recv_bytes;

public:
  UDP()
  {
    fd = -1;
    // close();
  }
  ~UDP()
  {
    // close();
  }

  bool open(int port = 0, bool share_port_for_multicasting = false, bool multicast_include_localhost = false,
            bool blocking = false)
  {
    const int TTL = 32;

    // open the socket
    if (fd >= 0)
      ::close(fd);
    fd = socket(PF_INET, SOCK_DGRAM, 0);

    // set socket as non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
      flags = 0;
    fcntl(fd, F_SETFL, flags | (blocking ? 0 : O_NONBLOCK));

    if (share_port_for_multicasting)
    {
      int reuse = 1;
      if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) != 0)
      {
        fprintf(stderr, "ERROR WHEN SETTING SO_REUSEADDR ON UDP SOCKET\n");
        fflush(stderr);
      }
      /*if(setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, 1)!=0) {
        fprintf(stderr,"ERROR WHEN SETTING SO_REUSEPORT ON UDP SOCKET\n");
        fflush(stderr);
      }*/
    }

    if (multicast_include_localhost)
    {
      int yes = 1;
      // allow packets to be received on this host
      if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&yes, sizeof(yes)) != 0)
      {
        fprintf(stderr, "ERROR WHEN SETTING IP_MULTICAST_LOOP ON UDP SOCKET\n");
        fflush(stderr);
      }
    }
    // sets the TTL value so routing of the packet is possible, if needed.
    int ret = setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &TTL, sizeof(TTL));
    if (ret != 0)
    {
      printf("ERROR %d WHEN SETTING IP_MULTICAST_TTL\n", ret);
      return false;
    }

    // bind socket to port if nonzero
    if (port != 0)
    {
      sockaddr_in sockname;
      sockname.sin_family = AF_INET;
      sockname.sin_addr.s_addr = htonl(INADDR_ANY);
      sockname.sin_port = htons(port);
      bind(fd, (struct sockaddr*)(&sockname), sizeof(sockname));
    }

    /*
    // allow port reuse (for when a program is quickly restarted)
    // (not sure we really need this)
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    */

    return (true);
  }

  bool addMulticast(const Address& multiaddr, const Address& interface)
  {
    static const bool debug = false;
    struct ip_mreq imreq;
    imreq.imr_multiaddr.s_addr = multiaddr.getInAddr();
    imreq.imr_interface.s_addr = interface.getInAddr();

    if (debug)
    {
      printf("0x%08X 0x%08X\n", (unsigned)interface.getInAddr(), (unsigned)INADDR_ANY);
    }

    int ret = setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(imreq));
    if (debug)
      printf("ret=%d\n", ret);
    if (ret != 0)
      return false;

    // set multicast output interface
    ret = setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF, &imreq.imr_interface.s_addr, sizeof(imreq.imr_interface.s_addr));
    if (debug)
      printf("ret=%d\n", ret);

    return (ret == 0);
  }
  void close()
  {
    if (fd >= 0)
      ::close(fd);
    fd = -1;

    sent_packets = 0;
    sent_bytes = 0;
    recv_packets = 0;
    recv_bytes = 0;
  }
  bool isOpen() const
  {
    return (fd >= 0);
  }

  bool send(const void* data, int length, const Address& dest)
  {
    int len = sendto(fd, data, length, 0, &dest.addr, dest.addr_len);

    if (len > 0)
    {
      sent_packets++;
      sent_bytes += len;
    }

    return (len == length);
  }
  int recv(void* data, int length, Address& src)
  {
    src.addr_len = sizeof(src.addr);
    int len = recvfrom(fd, data, length, 0, &src.addr, &src.addr_len);

    if (len > 0)
    {
      recv_packets++;
      recv_bytes += len;
    }

    return (len);
  }
  bool wait(int timeout_ms = -1) const
  {
    pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    return (poll(&pfd, 1, timeout_ms) == 1);
  }
  bool havePendingData() const
  {
    return (wait(0));
  }

  int getFd() const
  {
    return (fd);
  }
};
};  // namespace Net

/////////////////// END OF SSL-VISION CODE
///
///
///
///
///  MulticastClient Implementation based on SSL-Vision Code:

namespace rhoban_ssl
{
MulticastClientSingleThread2019::MulticastClientSingleThread2019(std::string addr, std::string port)
  : addr(addr), port(atoi(port.c_str()))
{
}

MulticastClientSingleThread2019::~MulticastClientSingleThread2019()
{
}

bool MulticastClientSingleThread2019::runTask()
{
  if (running == false)
    return false;

  for (unsigned int i = 0; i < nfds_; ++i)
    sockets_fds_[i].revents = 0;

  int e = poll(sockets_fds_, nfds_, 10);

  // printf("poll return %d \n", e);

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
        }
      }
    }
  }
  return true;
}

void MulticastClientSingleThread2019::init()
{
  Net::UDP mc;

  printf("MC 2019 init %s:%d\n", addr.c_str(), port);
  if (!mc.open(port, true, true, false))
  {
    fprintf(stderr, "Unable to open UDP network port: %d\n", port);
    fflush(stderr);
  }

  Net::Address multiaddr, interface;
  multiaddr.setHost(addr.c_str(), port);
  interface.setAny();

  printf("MC 2019 add mc\n", addr.c_str(), port);
  if (!mc.addMulticast(multiaddr, interface))
  {
    fprintf(stderr, "Unable to setup UDP multicast\n");
    fflush(stderr);
  }
  printf("MC 2019 getFd\n", addr.c_str(), port);
  int socketfd = mc.getFd();

  sockets_fds_ = new struct pollfd[1];
  nfds_ = 1;

  sockets_fds_[0].fd = socketfd;
  sockets_fds_[0].events = POLLIN;

  memset(msgs, 0, sizeof(msgs));
  for (int j = 0; j < VLEN; j++)
  {
    iovecs[j].iov_base = bufs[j];
    iovecs[j].iov_len = BUFSIZE;
    msgs[j].msg_hdr.msg_iov = &iovecs[j];
    msgs[j].msg_hdr.msg_iovlen = 1;
  }
  running = true;
}
}  // namespace rhoban_ssl
