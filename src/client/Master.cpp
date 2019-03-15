#include <iostream>
#include <unistd.h>
#include "Master.h"

namespace RhobanSSL
{
float Master::Robot::age()
{
  return diffSec(lastUpdate, rhoban_utils::TimeStamp::now());
}

bool Master::Robot::isOk()
{
  bool ok = present && age() < 0.5 && (status.status & STATUS_OK);
  // std::cout << "Status: " << (int)(status.status & STATUS_OK) << ", Present: "
  // << (int) present << ", Age: " << age() << " ok: " << (int)ok << std::endl;
  return ok;
}

Master::Robot::Robot()
{
  present = false;
}

Master::Master(std::string port, unsigned int baudrate) : serial(port, baudrate, serial::Timeout::simpleTimeout(1000))
{
  // em();

  shouldSend = false;
  shouldSendParams = false;
  running = true;
  thread = new std::thread([this]() { this->execute(); });
  tmpPacket = "";
  tmpNbRobots = 0;
  packet = "";
  nbRobots = 0;
  lastSend = rhoban_utils::TimeStamp::now();
}

Master::~Master()
{
  thread->join();
  delete thread;
}

void Master::em()
{
  // Resetting the communication structures
  struct packet_master packet;
  packet.actions = 0;
  packet.x_speed = 0;
  packet.y_speed = 0;
  packet.t_speed = 0;

  for (size_t k = 0; k < MAX_ROBOTS; k++)
  {
    robots[k].status.status = 0;
    addRobotPacket(k, packet);
  }
  send();
}

void Master::stop()
{
  running = false;
}

void Master::send()
{
  if (tmpPacket.size() > 0)
  {  // There is something to send
    // Waiting to either have received an answer from the last send() or reached the
    // overall cycle timeout
    bool waiting = true;
    while (waiting)
    {
      mutex.lock();
      if (receivedAnswer)
      {
        waiting = false;
      }
      else
      {
        // XXX: This timeout should be adjusted
        if (diffSec(lastSend, rhoban_utils::TimeStamp::now()) > 0.03)
        {
          waiting = false;
        }
      }
      mutex.unlock();
      usleep(100);
    }

    // XXX: Did we finished the last send cycle?
    // We should wait either the timeout or the reception is over
    packet = tmpPacket;
    nbRobots = tmpNbRobots;
    tmpPacket.clear();
    tmpNbRobots = 0;
    receivedAnswer = false;
    lastSend = rhoban_utils::TimeStamp::now();
    shouldSend = true;
  }
}

void Master::addPacket(int robot, int instruction, char* packet, size_t len)
{
  std::string data(packet, len);
  tmpPacket += (char)robot;
  tmpPacket += (char)instruction;
  tmpPacket += data;

  // Padding to complete until PACKET_SIZE
  for (size_t k = 0; k < PACKET_SIZE - len - 1; k++)
  {
    tmpPacket += (char)0;
  }
  tmpNbRobots++;
}

void Master::addRobotPacket(int robot, struct packet_master robotPacket)
{
  addPacket(robot, INSTRUCTION_MASTER, (char*)&robotPacket, sizeof(robotPacket));
}

void Master::addParamPacket(int robot, struct packet_params params)
{
  addPacket(robot, INSTRUCTION_PARAMS, (char*)&params, sizeof(params));
}

void Master::sendPacket()
{
  uint8_t data[packet.size() + 4];
  data[0] = 0xaa;
  data[1] = 0x55;
  data[2] = nbRobots;

  memcpy((void*)(data + 3), packet.c_str(), packet.size());
  data[sizeof(data) - 1] = 0xff;
  receivedAnswer = false;
  mutex.unlock();

  // Sending the data
  serial.write(data, sizeof(data));
}

void Master::execute()
{
  int state = 0;
  unsigned int pos = 0;
  size_t nb_robots = 0;
  uint8_t temp[1024];

  serial.write("master\nmaster\nmaster\n");

  while (running)
  {
    // XXX: Maybe we should use something like select() here
    usleep(100);
    size_t n = serial.available();

    if (n)
    {
      uint8_t buffer[n];
      serial.read(buffer, n);
      for (size_t k = 0; k < n; k++)
      {
        uint8_t c = buffer[k];

        if (state == 0)
        {
          if (c == 0xaa)
          {
            state++;
          }
        }
        else if (state == 1)
        {
          if (c == 0x55)
          {
            state++;
            pos = 0;
          }
          else
          {
            state = 0;
          }
        }
        else if (state == 2)
        {
          nb_robots = c;
          pos = 0;
          if (nb_robots <= MAX_ROBOTS)
          {
            state++;
          }
          else
          {
            state = 0;
          }
        }
        else if (pos < nb_robots * (1 + sizeof(struct packet_robot)))
        {
          temp[pos++] = c;
        }
        else
        {
          if (c == 0xff)
          {
            /*
            if (nb_robots == 0) {
                std::cout << "- No robots in answer!" << std::endl;
            } else {
                std::cout << "- Robots in answer!" << std::endl;
            }
            */
            /*
            if (nb_robots > 0) {
                static int packets = 0;
                packets++;
                printf("Packets: %d\n", packets);
            } else {
                printf("No robots!");
            }
            */
            // Received message from USB, reading the status of each robot
            mutex.lock();
            receivedAnswer = true;
            for (size_t k = 0; k < nb_robots; k++)
            {
              int robot_id = temp[k * (1 + sizeof(struct packet_robot))];
              if (robot_id < MAX_ROBOTS)
              {
                memcpy(&robots[robot_id].status, &temp[k * (1 + sizeof(struct packet_robot)) + 1],
                       sizeof(struct packet_robot));
                robots[robot_id].present = true;
                robots[robot_id].lastUpdate = rhoban_utils::TimeStamp::now();
              }
            }
            mutex.unlock();
          }
          state = 0;
        }
      }
    }

    mutex.lock();
    if (shouldSend)
    {
      shouldSend = false;
      sendPacket();
    }
    else
    {
      mutex.unlock();
    }
  }
}
}  // namespace RhobanSSL
