#pragma once

#include <mutex>
#include <thread>
#include <rhoban_utils/timing/time_stamp.h>
#include <messages_robocup_ssl_wrapper.pb.h>
#include "MulticastClient.h"

namespace RhobanSSL
{
class VisionClient : public MulticastClient
{
public:
    VisionClient(bool simulation = false);
    bool process(char *buffer, size_t len);
    SSL_WrapperPacket getData();

protected:
    SSL_WrapperPacket data;
};
}
