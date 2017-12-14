#include <chrono>

#include <iostream>
#include "VisionClient.h"
#include "client_config.h"

using namespace Utils::Timing;


namespace RhobanSSL
{
    VisionClient::VisionClient() :
    MulticastClient(SSL_VISION_ADDRESS, SSL_VISION_PORT)
    {
    }

    SSL_WrapperPacket VisionClient::getData()
    {
        SSL_WrapperPacket tmp;

        mutex.lock();
        tmp = data;
        mutex.unlock();

        return tmp;
    }

    bool VisionClient::process(char *buffer, size_t len)
    {
        SSL_WrapperPacket packet;

        if (packet.ParseFromArray(buffer, len)) {
            data = packet;

            return true;
        } else {
            std::cerr << "Packet error!" << std::endl;
        }

        return false;
    }
}
