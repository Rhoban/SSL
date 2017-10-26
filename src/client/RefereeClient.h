#pragma once

#include <mutex>
#include <thread>
#include <timing/TimeStamp.hpp>
#include <referee.pb.h>
#include "MulticastClient.h"

namespace RhobanSSL
{
class RefereeClient : public MulticastClient
{
public:
    struct Interface {
        std::string name;
        int family;
        int index;
    };

    RefereeClient();
    bool process(char *buffer, size_t len);
    SSL_Referee getData();

protected:
    SSL_Referee data;
};
}
