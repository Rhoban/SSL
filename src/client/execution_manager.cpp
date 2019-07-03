#include "execution_manager.h"

#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>

namespace rhoban_ssl
{
ExecutionManager ExecutionManager::execution_manager_singleton_;

ExecutionManager::ExecutionManager() : shutdown_(false), current_max_priority_(100)
{
}

ExecutionManager& ExecutionManager::getManager()
{
  return ExecutionManager::execution_manager_singleton_;
}

void ExecutionManager::addTask(Task* t, int priority)
{
  if (priority == -1)
  {
    priority = current_max_priority_;
    current_max_priority_ += 1;
  }

  add_buffer_.insert(std::pair<int, Task*>(priority, t));
}

void ExecutionManager::run(double min_loop_duration)
{
  using std::chrono::high_resolution_clock;
  std::vector<std::pair<int, Task*> > to_remove;
  high_resolution_clock::time_point start;
  long min_loop_d = long(min_loop_duration * 1e9);
  do
  {
    start = high_resolution_clock::now();
    for (auto i : add_buffer_)
    {
      tasks_.insert(i);
    }
    add_buffer_.clear();
    to_remove.clear();
    for (auto i : tasks_)
    {
      if (i.second->runTask() == false)
      {
        to_remove.push_back(i);
      }
    }
    for (auto i : to_remove)
    {
      tasks_.erase(i);
      delete i.second;
    }

    long loop_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(high_resolution_clock::now() - start).count();
    long d = min_loop_d - loop_duration;
    ;
    if (d > 0)
    {
      // std::cout << d << std::endl;
      std::this_thread::sleep_for(std::chrono::nanoseconds(d));
    }
  } while ((tasks_.size() > 0) && (shutdown_ == false));

  std::cout << std::endl << "DELETING TASKS" << std::endl;
  std::cout << "----------------------" << std::endl;
  for (auto i = tasks_.begin(); i != tasks_.end(); ++i)
    delete i->second;
  std::cout << "----------------------" << std::endl;
  std::cout << "END" << std::endl;
}

void ExecutionManager::shutdown()
{
  shutdown_ = true;
}

Task::~Task()
{
}

TimeStatTask::TimeStatTask(int print_freq) : last_times(print_freq), counter_(0), print_freq_(print_freq)
{
}

bool TimeStatTask::runTask()
{
  using std::chrono::high_resolution_clock;
  last_times[counter_] = high_resolution_clock::now();
  counter_ += 1;
  if (counter_ == print_freq_)
  {
    long durations = 0, min = 1000000000, max = -1;
    for (int i = 0; i < (counter_ - 1); ++i)
    {
      long d = std::chrono::duration_cast<std::chrono::nanoseconds>(last_times[i + 1] - last_times[i]).count();
      durations += d;
      if (d > max)
        max = d;
      if (d < min)
        min = d;
    }
    double average = (((double)durations) / ((double)print_freq_ - 1)) / 1000000000.0;
    double dmin = ((double)min) / 1000000000.0;
    double dmax = ((double)max) / 1000000000.0;
    std::cout << "execution manager loop stat: " << dmin << " " << average << " " << dmax << std::endl;
    counter_ = 0;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////

rhoban_ssl::ConditionalTask::ConditionalTask(const std::function<bool()>& condition, const std::function<bool()>& job)
  : condition_(condition), job_(job)
{
}

bool ConditionalTask::runTask()
{
  if (condition_())
  {
    return job_();
  }
  return true;
}

}  // namespace rhoban_ssl
