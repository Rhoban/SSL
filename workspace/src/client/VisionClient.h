#pragma once

#include <mutex>
#include <thread>
#include <timing/TimeStamp.hpp>
#include <messages_robocup_ssl_wrapper.pb.h>
#include "MulticastClient.h"

namespace RhobanSSL
{
class VisionClient : public MulticastClient
{
public:
    struct Interface {
        std::string name;
        int family;
        int index;
    };

    VisionClient();
    bool process(char *buffer, size_t len);
    SSL_WrapperPacket getData();

protected:
    SSL_WrapperPacket data;
};
}
