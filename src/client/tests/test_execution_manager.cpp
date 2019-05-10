/*
    This file is part of SSL.

    Copyright TODO : Julien Allali.
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com) (Refacto)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <gtest/gtest.h>
#include <execution_manager.h>
#include <chrono>
#include <iostream>
#include <assert.h>
#include <multicast_client_single_thread.h>
#include <google/protobuf/stubs/common.h>

static int counter = 0;
class SimpleTask : public virtual rhoban_ssl::Task
{
  int ncalls;
  int& calls;

public:
  SimpleTask(int ncalls, int& count) : ncalls(ncalls), calls(count)
  {
    calls = 0;
  }
  virtual bool runTask() override
  {
    counter += 1;
    calls += 1;
    if (calls >= ncalls)
      return false;
    return true;
  }
};

TEST(test_execution_manager, add_task)
{
  int simple1 = 0, simple2 = 0;

  using std::chrono::high_resolution_clock;
  high_resolution_clock::time_point start = high_resolution_clock::now();
  rhoban_ssl::ExecutionManager::getManager().addTask(new SimpleTask(100, simple1));
  rhoban_ssl::ExecutionManager::getManager().addTask(new SimpleTask(200, simple2));
  rhoban_ssl::ExecutionManager::getManager().run(0.01);
  double loop_duration = std::chrono::duration<double>(high_resolution_clock::now() - start).count();
  EXPECT_TRUE(simple1 == 100);
  EXPECT_TRUE(simple2 == 200);
  EXPECT_TRUE(counter == 300);
  EXPECT_TRUE((loop_duration >= 2) && (loop_duration <= 2.2));
}

/*
class NetTest : public rhobanssl::MulticastClientSingleThread
{
public:
  int ncalls;

  NetTest(const std::string address, const std::string& port) : rhobanssl::MulticastClientSingleThread(address, port)
  {
    ncalls = 0;
    init();
  }
  virtual bool process(char* buffer, size_t len) override
  {
    std::cout << "packet received" << std::endl;
    ncalls += 1;
    if (ncalls > 10)
      shutdown();
    return true;
  }
};

  NetTest* n = new NetTest(argv[1], argv[2]);
  rhobanssl::ExecutionManager::getManager().addTask(n);
  rhobanssl::ExecutionManager::getManager().run(0.01);
*/

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  ::google::protobuf::ShutdownProtobufLibrary();  // clear global data allocated by protobuf
  return 0;
}
