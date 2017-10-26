#pragma once

#include <vector>
#include <mutex>
#include <thread>
#include <timing/TimeStamp.hpp>

namespace RhobanSSL
{
class MulticastClient
{
public:
    struct Interface {
        std::string name;
        int family;
        int index;
    };

    MulticastClient(std::string addr, std::string port);
    virtual ~MulticastClient();

    void run(int family, int ifindex);
    virtual bool process(char *buffer, size_t len)=0;
    virtual void hasPacket();
    bool hasData();

protected:
    std::string addr, port;
    std::vector<std::thread*> threads;
    std::mutex mutex;
    bool receivedData;
    volatile bool running;
    Utils::Timing::TimeStamp lastData;
};
}
