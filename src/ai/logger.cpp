#include "logger.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdio.h>
#include <config.h>
#include <sys/select.h>

namespace rhoban_ssl
{
LoggerTask::LoggerTask(ai::AI* ai, std::string filename, int max_file_size)
  : ai_(ai), log_memory_(nullptr), max_memory_(0), current_position_(0), mmap_fd_(0), counter_(0), filename_(filename)
{
  mmap_fd_ = open(filename_.c_str(), O_CREAT | O_TRUNC | O_RDWR, 0666);
  if (mmap_fd_ == -1)
  {
    perror(filename_.c_str());
    return;
  }
  if (lseek(mmap_fd_, max_file_size, SEEK_SET) == -1)
  {
    perror("seek on log file");
    close(mmap_fd_);
    return;
  }
  char c = 0;
  write(mmap_fd_, &c, 1);
  lseek(mmap_fd_, 0, SEEK_SET);
  void* data = mmap(nullptr, max_file_size, PROT_WRITE | PROT_READ, MAP_SHARED, mmap_fd_, 0);
  if (data == MAP_FAILED)
  {
    perror("mmap failed:");
    return;
  }
  log_memory_ = (char*)data;
  max_memory_ = max_file_size;
  *(static_cast<int*>(data)) = 0;
  if (ai_ != nullptr)
    *((int*)log_memory_ + sizeof(int)) = 1;
  else
    *((int*)log_memory_ + sizeof(int)) = 0;
  current_position_ = 2 * sizeof(int);
}

LoggerTask::~LoggerTask()
{
  if (log_memory_ != nullptr)
  {
    munmap((void*)log_memory_, max_memory_);
    close(mmap_fd_);
    truncate(filename_.c_str(), current_position_);
  }
}

bool LoggerTask::runTask()
{
  DEBUG("logger into " << filename_ << " current position is " << current_position_ << " max is " << max_memory_);
  if ((log_memory_ == nullptr) || (current_position_ + 5000) > max_memory_)
    return false;
  *((int*)log_memory_) = *((int*)log_memory_) + 1;
  if (ai_ != nullptr)
  {
    memcpy(log_memory_ + current_position_, (void*)ai_, sizeof(ai::AI));
    current_position_ += sizeof(ai::AI);
  }
  memcpy(log_memory_ + current_position_, (void*)Data::get(), sizeof(Data));
  current_position_ += sizeof(Data);
  return true;
}

void LogReplayTask::load(int frame)
{
  if (has_ai_)
  {
    int memshift = 2 * sizeof(int) + sizeof(ai::AI) + frame * (sizeof(Data) + sizeof(ai::AI));
    DEBUG("load Data with a shift of " << memshift);
    memcpy((void*)Data::get(), (void*)(log_address_ + memshift), sizeof(Data));
  }
  else
  {
    int memshift = 2 * sizeof(int) + sizeof(ai::AI) + frame * (sizeof(Data));
    DEBUG("load Data with a shift of " << memshift);
    memcpy((void*)Data::get(), (void*)(log_address_ + memshift), sizeof(Data));
  }
  DEBUG("ball last update is " << Data::get()->ball.last_update);
  DEBUG("ball address " << (void*)&(Data::get()->ball) << " size is " << sizeof(Data::get()->ball));
}

LogReplayTask::LogReplayTask(std::string filename) : log_address_(nullptr), mmap_fd_(-1), current_(0)
{
  mmap_fd_ = open(filename.c_str(), O_RDONLY);
  if (mmap_fd_ == -1)
  {
    perror(filename.c_str());
    exit(1);
  }
  int s = lseek(mmap_fd_, 0, SEEK_END);
  lseek(mmap_fd_, 0, SEEK_SET);
  void* m = mmap(nullptr, s, PROT_READ, MAP_PRIVATE, mmap_fd_, 0);
  if (m == MAP_FAILED)
  {
    perror("mmap failed!");
    exit(1);
  }
  log_address_ = (char*)m;
  nb_frames_ = *((int*)log_address_);
  has_ai_ = *(((int*)log_address_ + sizeof(int)));
  DEBUG("found " << nb_frames_ << " entries into log file, ai: " << has_ai_);
  ai::Config::log_replay = true;
  load(0);

  struct termios term;
  tcgetattr(0, &termsave);
  term = termsave;
  term.c_lflag = term.c_lflag & ~ICANON;
  tcsetattr(0, TCSANOW, &term);
}

LogReplayTask::~LogReplayTask()
{
  tcsetattr(0, TCSANOW, &termsave);
}

bool LogReplayTask::runTask()
{
  DEBUG("log replay is on frame " << current_);
  if (current_ >= nb_frames_)
    current_ = nb_frames_ - 1;
  fd_set rset;
  FD_ZERO(&rset);
  FD_SET(0, &rset);
  struct timeval now;
  now.tv_sec = 0;
  now.tv_usec = 0;
  select(1, &rset, nullptr, nullptr, &now);
  if (FD_ISSET(0, &rset))
  {
    char action;
    read(0, &action, sizeof(char));
    printf("chr read is %d (%c) %x", action, action, action);
    if (action == 'p')
      current_ += 1;
    if (action == 'o')
      current_ -= 1;
    if (action == 'm')
      current_ += 10;
    if (action == 'l')
      current_ -= 10;
    if (action == 'q')
      ExecutionManager::getManager().shutdown();
  }

  if (current_ < 0)
    current_ = 0;
  if (current_ >= nb_frames_)
    current_ = nb_frames_ - 1;

  load(current_);

  return true;
}
}
