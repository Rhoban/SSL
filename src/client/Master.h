#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <serial/serial.h>
#include "structs.h"

namespace RhobanSSL
{
class Master
{
public:
    Master(std::string port, unsigned int baudrate);
    virtual ~Master();

    // Emergency stop
    void em();

    // Stop the master
    void stop();

    // Send the packet
    void send();

    volatile struct packet_master robots[6];
    volatile struct packet_robot statuses[6];

protected:
    bool running;
    bool shouldSend;

    serial::Serial serial;
    std::thread *thread;
    std::mutex mutex;


    void execute();
};
}
