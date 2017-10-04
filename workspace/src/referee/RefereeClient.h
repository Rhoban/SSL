#pragma once

#include <mutex>
#include <thread>
#include <timing/TimeStamp.hpp>
#include <referee.pb.h>

namespace RhobanSSL
{
class RefereeClient
{
public:

    struct Interface {
        std::string name;
        int family;
        int index;
    };

    RefereeClient();
    virtual ~RefereeClient();

    void run(int family, int ifindex);
    SSL_Referee getData();
    bool hasData();

protected:
    SSL_Referee data;
    std::vector<std::thread*> threads;
    std::mutex mutex;
    bool receivedData;
    volatile bool running;
    Utils::Timing::TimeStamp lastData;
};
}
