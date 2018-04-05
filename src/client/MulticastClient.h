#pragma once

#include <vector>
#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>

namespace RhobanSSL
{

/**
 * This is a generic multicast client that listens on all possible interfaces
 * (running one thread per interface) for incoming packets
 */
class MulticastClient
{
public:
    struct Interface {
        std::string name;
        int family;
        int index;
    };

    /**
     * At instanciation, provide the address and the port to listen to for the
     * multicast client to listen to
     *
     * @param addr Multicast address to listen
     * @param port Multicast port to listen
     */
    MulticastClient(std::string addr, std::string port);
    virtual ~MulticastClient();

    /**
     * Process an incoming packet
     *
     * @param  buffer  A buffer containing received bytes
     * @param  len     Number of bytes in the buffer
     * @return         Returns true if the contents is a valid packet
     */
    virtual bool process(char *buffer, size_t len)=0;

    /**
     * Is there valid received packet ?
     */
    bool hasData();

    /**
     * How many packets were received ?
     */
    unsigned int getPackets();

protected:
    std::string addr, port;
    std::vector<std::thread*> threads;
    std::mutex mutex;
    bool receivedData;
    volatile bool running;
    rhoban_utils::TimeStamp lastData;
    unsigned int packets;

    /**
     * Initializes the multicast client
     */
    void init();

    /**
     * Each thread runs this process
     *
     * @param family  Interface family
     * @param ifindex Interface index
     */
    void run(int family, int ifindex);

    /**
     * Called when a valid packet incomes, can be overloaded in sub classes
     * to trigger events when a packet is received
     */
    virtual void packetReceived();
};
}
