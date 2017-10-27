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
    RefereeClient();
    bool process(char *buffer, size_t len);
    SSL_Referee getData();

protected:
    SSL_Referee data;
};
}
