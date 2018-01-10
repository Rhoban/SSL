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
            robots[k].wheel1 = 0;
            robots[k].wheel2 = 0;
            robots[k].wheel3 = 0;
            robots[k].wheel4 = 0;

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
                uint8_t data[sizeof(robots) + 3];
                data[0] = 0xaa;
                data[1] = 0x55;
                data[sizeof(robots) + 2] = 0xff;
                memcpy((void *)(data + 2), (void *)robots, sizeof(robots));
                mutex.unlock();

                serial.write(data, sizeof(data));
            } else {
                mutex.unlock();
            }
        }
    }
}
