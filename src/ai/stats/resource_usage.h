/*
  This file is part of SSL.

  Copyright 2019 Jérémy Bezamat (jeremy.bezamat@gmail.com)
  Copyright 2019 Mael Keryell-Even (keryelleven.mael@gmail.com)

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

#include <execution_manager.h>
#include <core/gnu_plot.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <chrono>

namespace rhoban_ssl
{
namespace stats
{
class ResourceUsage : public Task
{
private:
  bool plot_info_;
  bool print_info_;
  int read_every_n_loop_;

  typedef struct SelfCPUData
  {
    int pid;
    char com[30];
    char state;
    int ppid;
    int pgrp;
    int session;
    int tty_nr;
    int tpgid;
    unsigned int flags;
    long unsigned int minflt;
    long unsigned int cminlt;
    long unsigned int majflt;
    long unsigned int cmajflt;
    long unsigned int utime;
    long unsigned int stime;
    long int cutime;
    long int cstime;
    long int priority;
    long int nice;
    long int num_threads;
    long int itrealvalue;
    long long unsigned int starttime;
    long unsigned int vsize;
    long int rss;
    long unsigned int rsslim;
    long unsigned int startcode;
    long unsigned int endcode;
    long unsigned int startstack;
    long unsigned int kstkesp;
    long unsigned int kstkeip;
    long unsigned int signal;
    long unsigned int blocked;
    long unsigned int sigignore;
    long unsigned int sigcatch;
    long unsigned int wchan;
    long unsigned int nswap;
    long unsigned int cnswap;
    int exit_signal;
    int processor;
    unsigned int rt_priority;
    unsigned int policy;
    long long unsigned int delayacct_blkio_ticks;
    long unsigned int guest_time;
    long int cguest_time;
    long unsigned int start_data;
    long unsigned int end_data;
    long unsigned int start_brk;
    long unsigned int arg_start;
    long unsigned int arg_end;
    long unsigned int env_start;
    long unsigned int env_end;
    int exit_code;
  } SelfCPUData;

  SelfCPUData self_cpu_data_;
  SelfCPUData old_self_cpu_data_;

  GnuPlot plot_;
  int loop_count_ = 0;

  std::chrono::high_resolution_clock::time_point time_now_;
  std::chrono::high_resolution_clock::time_point previous_time_;
  long unsigned int utime_ = 0;
  long unsigned int previous_utime_;

  bool first_time_ = true;
  int warmup = 0;

  unsigned int max_diff_vsize_displayed_ = 60;
  unsigned int max_diff_minflt_displayed_ = 70;

public:
  /**
   * @brief Construct a new Resource Usage object
   *
   * @param read_every_n_loop_
   */
  ResourceUsage(int read_every_n_loop_ = 50);

  /**
   * @brief Construct a new Resource Usage object
   *
   * @param plot_info : true -> plot of cpu info
   * @param print_inf : true -> print cpu info (DEBUG)
   * @param read_every_n_loop_
   */
  ResourceUsage(bool plot_info = true, bool print_info = false, int read_every_n_loop_ = 50);

  /**
   * @brief gets values of rusage (...) and plot them
   *
   * @return true : continue the runTask
   * @return false : stop the runTask
   */
  virtual bool runTask() override;

  /**
   * @brief
   *
   * @return int (-1 can't read /proc/self/stat)
   */
  int readStatsSelfCPU();

  /**
   * @brief
   *
   */
  void printStatFile();

  /**
   * @brief Destroy the Task object
   *
   */
  ~ResourceUsage();
};

}  // namespace stats
}  // namespace rhoban_ssl
