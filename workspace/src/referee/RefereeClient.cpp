#include <chrono>
#include <ifaddrs.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <iostream>
#include "RefereeClient.h"

using namespace Utils::Timing;

#define SSL_BROADCAST_PORT "10003"
#define SSL_BROADCAST_ADDR "224.5.23.1"

namespace RhobanSSL
{

    RefereeClient::RefereeClient() :
    receivedData(false)
    {
        // Listing interfaces
        std::map<int, Interface> interfaces;
        ifaddrs *ifs = 0;
        if (getifaddrs(&ifs) < 0) {
            std::cerr << "Can't get network interface list" << std::endl;
            exit(0);
        } try {
            for (const ifaddrs *i = ifs; i; i = i->ifa_next) {
                if ((i->ifa_flags & IFF_UP) && (i->ifa_flags & IFF_MULTICAST) && i->ifa_addr) {
                    if (i->ifa_addr->sa_family == AF_INET /*|| i->ifa_addr->sa_family == AF_INET6*/) {
                        unsigned int ifindex = if_nametoindex(i->ifa_name);
                        if (ifindex) {
                            Interface interface;
                            interface.name = i->ifa_name;
                            interface.family = i->ifa_addr->sa_family;
                            interface.index = ifindex;
                            interfaces[ifindex] = interface;
                        }
                    }
                }
            }
            freeifaddrs(ifs);
        } catch (...) {
            freeifaddrs(ifs);
            throw;
        }

        // For each interface, running a thread
        running = true;
        for (auto entry : interfaces) {
            auto interface = entry.second;
            std::thread *thread = new std::thread([this, interface]() {
                run(interface.family, interface.index);
            });

            threads.push_back(thread);
        }
    }

    RefereeClient::~RefereeClient()
    {
        running = false;

        for (auto thread : threads) {
            thread->join();
            delete thread;
        }
    }

    void RefereeClient::run(int family, int ifindex)
    {
        // This is mainly from ssl-refbox/client example

        // Get a socket address to bind to.
        addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = 0;
        hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;
        addrinfo *ai = 0;
        int gai_err;
        if ((gai_err = getaddrinfo(0, SSL_BROADCAST_PORT, &hints, &ai)) != 0) {
            std::cerr << gai_strerror(gai_err);
            return;
        }

        // Create and bind a socket.
        int sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock < 0) {
            std::cerr << strerror(errno) << '\n';
            return;
        }
        if (bind(sock, ai->ai_addr, ai->ai_addrlen) < 0) {
            std::cerr << strerror(errno) << '\n';
            return;
        }

        // Join the multicast group.
        ip_mreqn mcreq;
        mcreq.imr_multiaddr.s_addr = inet_addr(SSL_BROADCAST_ADDR);
        mcreq.imr_address.s_addr = INADDR_ANY;
        mcreq.imr_ifindex = ifindex;
        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mcreq, sizeof(mcreq)) < 0) {
                std::cerr << strerror(errno) << '\n';
                return;
        }

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000;

        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                    sizeof(timeout));
        setsockopt (sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                    sizeof(timeout));

        while (running) {
            uint8_t buffer[65536];
            ssize_t len = recv(sock, buffer, sizeof(buffer), 0);

            if (len > 0) {
                SSL_Referee packet;
                if (packet.ParseFromArray(buffer, len)) {
                    mutex.lock();
                    receivedData = true;
                    lastData = TimeStamp::now();
                    data = packet;
                    mutex.unlock();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool RefereeClient::hasData()
    {
        if (receivedData) {
            auto delta = diffMs(lastData, TimeStamp::now());
            return delta < 3000;
        }

        return false;
    }

    SSL_Referee RefereeClient::getData()
    {
        SSL_Referee tmp;

        mutex.lock();
        tmp = data;
        mutex.unlock();

        return tmp;
    }

}
