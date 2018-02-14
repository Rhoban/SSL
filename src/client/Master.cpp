#include <iostream>
#include <unistd.h>
#include "Master.h"

namespace RhobanSSL
{
    Master::Master(std::string port, unsigned int baudrate)
    : serial(port, baudrate, serial::Timeout::simpleTimeout(1000))
    {
        em();

        shouldSend = false;
        shouldSendParams = false;
        running = true;
        thread = new std::thread([this]() {
            this->execute();
        });
    }

    Master::~Master()
    {
        thread->join();
        delete thread;
    }

    void Master::em()
    {
        // Resetting the communication structures
        for (int k = 0; k < 6; k++) {
            robots[k].actions = 0;
            robots[k].x_speed = 0;
            robots[k].y_speed = 0;
            robots[k].t_speed = 0;

            statuses[k].status = 0;
        }

        shouldSend = true;
    }

    void Master::stop()
    {
        running = false;
    }

    void Master::send()
    {
        shouldSend = true;
    }

    void Master::setParams(float kp, float ki, float kd)
    {
        params.kp = kp;
        params.ki = ki;
        params.kd = kd;
        shouldSendParams = true;
    }

    void Master::sendPacket(uint8_t instruction, uint8_t *payload, size_t size)
    {
        uint8_t data[size + 4];
        data[0] = 0xaa;
        data[1] = 0x55;
        data[2] = instruction;

        data[size + 3] = 0xff;
        memcpy((void *)(data + 3), payload, size);
        mutex.unlock();

        serial.write(data, sizeof(data));
    }

    void Master::execute()
    {
        int state = 0;
        int pos = 0;
        uint8_t temp[sizeof(statuses)];

        serial.write("master\nmaster\nmaster\n");

        while (running) {
            usleep(500);

            size_t n = serial.available();
            if (n) {
                uint8_t buffer[n];
                serial.read(buffer, n);
                for (size_t k = 0; k < n; k++) {
                    uint8_t c = buffer[k];

                    if (state == 0) {
                        if (c == 0xaa) {
                            state++;
                        }
                    } else if (state == 1) {
                        if (c == 0x55) {
                            state++;
                            pos = 0;
                        } else {
                            state = 0;
                        }
                    } else if (pos < sizeof(statuses)) {
                        temp[pos++] = c;
                    } else {
                        if (c == 0xff) {
                            // Received message from USB
                            mutex.lock();
                            memcpy((void *)statuses, (void *)temp, sizeof(statuses));
                            mutex.unlock();
                        }
                        state = 0;
                    }
                }
            }

            mutex.lock();
            if (shouldSend) {
                shouldSend = false;

                sendPacket(
                    INSTRUCTION_MASTER,
                    (uint8_t *)robots,
                    sizeof(robots)
                );

            } else if (shouldSendParams) {
                shouldSendParams = false;

                sendPacket(
                    INSTRUCTION_PARAMS,
                    (uint8_t *)&params,
                    sizeof(params)
                );
            } else {
                mutex.unlock();
            }
        }
    }
}
