#pragma once

#include <execution_manager.h>
#include <string>
#include "ai.h"
#include <termios.h>
#include <unistd.h>

namespace rhoban_ssl
{
class LoggerTask : public Task
{
  ai::AI* ai_;
  char* log_memory_;
  int max_memory_;
  int current_position_;
  int mmap_fd_;
  int counter_;
  std::string filename_;

  int memsum;

  std::vector<std::pair<int*, int>> watched_mem;
  int computeDataMemSum();

public:
  LoggerTask(ai::AI* ai, std::string filename, int max_file_size);
  virtual ~LoggerTask();
  virtual bool runTask() override;
};

class LogReplayTask : public Task
{
  char* log_address_;
  int mmap_fd_;
  bool has_ai_;
  int current_;
  int nb_frames_;
  struct termios termsave;
  ai::AI* aiptr_;
  void load(int frame);

public:
  LogReplayTask(std::string filename, ai::AI* aiptr = nullptr);
  virtual ~LogReplayTask();
  virtual bool runTask() override;
};
}
