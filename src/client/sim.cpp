#include <iostream>
#include <cmath>
#include <sstream>
#include <unistd.h>
#include <signal.h>
#include "SimClient.h"

static volatile bool running = true;

void stop(int s)
{
  running = false;
}

int main()
{
  signal(SIGINT, stop);
  rhoban_ssl::SimClient client;

  std::cout << "Putting ball on the middle..." << std::endl;
  client.moveBall(0, 0, 0, 0);
  client.moveRobot(1, 3, 0.5, 0, 180, 1);

  usleep(2000000);

  std::cout << "Setting robot yellow 3 at 1m/s..." << std::endl;
  client.send(1, 3, 1, 0, 0, 0, 0, false);

  usleep(1000000);

  std::cout << "Kicking!" << std::endl;
  client.send(1, 3, 0, 0, 0, 10, 0, false);

  usleep(1000000);

  std::cout << "Setting robot yellow 3 at -1m/s..." << std::endl;
  client.send(1, 3, -1, 0, 0, 0, 0, false);

  usleep(1000000);

  std::cout << "Stopping robot yellow..." << std::endl;
  client.send(1, 3, 0, 0, 0, 0, 0, false);

  usleep(1000000);
}
