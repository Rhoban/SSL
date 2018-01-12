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

    // Set params
    void setParams(float kp, float ki, float kd);

    // Master packets and statuses
    volatile struct packet_master robots[6];
    volatile struct packet_robot statuses[6];
    volatile struct packet_params params;

protected:
    bool running;
    bool shouldSend;
    bool shouldSendParams;

    serial::Serial serial;
    std::thread *thread;
    std::mutex mutex;

    void execute();
    void sendPacket(uint8_t instruction, uint8_t *payload, size_t size);
};
}
