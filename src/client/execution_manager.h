#pragma once

#include <set>
#include <vector>
#include <chrono>
#include <functional>

namespace rhoban_ssl
{
/**
 * @brief The Task class is an interface for job to be executed by the manager
 */
class Task
{
public:
  virtual ~Task();
  virtual bool runTask(void) = 0;
};

class TimeStatTask : public Task
{
  std::vector<std::chrono::high_resolution_clock::time_point> last_times;
  int counter_;
  int print_freq_;

public:
  TimeStatTask(int print_freq);
  virtual bool runTask(void) override;
};

class ConditionalTask : public Task
{
  std::function<bool()> condition_;
  std::function<bool()> job_;

public:
  ConditionalTask(const std::function<bool()>& condition, const std::function<bool()>& job);

  // Task interface
public:
  bool runTask();
};

class TimeoutTask : public Task
{
  double duration;
  std::chrono::high_resolution_clock::time_point start;

public:
  TimeoutTask(double d);
  virtual bool runTask(void) override;
};

/**
 * @brief The ExecutionManager class is used to register tasks and execute them in a loop
 */
class ExecutionManager
{
  std::set<std::pair<int, Task*>> tasks_;       // tasks actually executed in run loop
  std::set<std::pair<int, Task*>> add_buffer_;  // need if task are added during the run loop
  ExecutionManager();
  static ExecutionManager execution_manager_singleton_;
  bool shutdown_;
  int current_max_priority_;

public:
  static ExecutionManager& getManager();
  /**
   * @brief addTask add a task to the manager. This new Task must be allocated with new as it will be deallocated with
   * delete
   */
  void addTask(Task*, int priority = -1);
  /**
   * @brief run Run all task in a loop until all tasks auto-removed
   * @param min_loop_duration : minimum time between to loops over registered tasks
   */
  void run(double min_loop_duration);
  void shutdown();
};
}  // namespace rhoban_ssl
