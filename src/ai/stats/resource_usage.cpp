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

#include "resource_usage.h"
#include <debug.h>
#include <ctime>
#include <iostream>
#include <thread>
#include <data.h>

namespace rhoban_ssl
{
namespace stats
{
ResourceUsage::ResourceUsage()
{
}

bool ResourceUsage::runTask()
{
  using namespace std::chrono;
  if(loop_count_ == 10)
  {
    if(readStatsSelfCPU() == 0){
      previous_time_ = time_now_;
      time_now_ = high_resolution_clock::now();
      long loop_duration = duration_cast<std::chrono::nanoseconds>(time_now_ - previous_time_).count();
      
      previous_utime_ = utime_;
      utime_ = self_cpu_data_.utime;

      if(loop_duration == 0){
        return true;
      }

      plot_.setX(Data::get()->ai_data.time);
      plot_.push("num_threads", self_cpu_data_.num_threads);
      plot_.push("Virtual memory size (bytes*1e-8)", self_cpu_data_.vsize * 1e-8);
      // plot_.push("rss", self_cpu_data_.rss);
      plot_.push("CPU (%)", ( (100 * ((double)(utime_ - previous_utime_)/sysconf(_SC_CLK_TCK)))/ (loop_duration * 1e-9)));
      plot_.render();
    }
    loop_count_ = 0;
  }
  
  loop_count_++;
  return true;
}

int ResourceUsage::readStatsSelfCPU()
{
  FILE* stat_file;
  if((stat_file = fopen("/proc/self/stat", "r")) == NULL)
  {
    DEBUG("can't read /proc/self/stat");
    return -1;
  }

  fscanf(stat_file , "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu\
  %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu\
  %lu %lu %lu %lu %d %d %u %u %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %d",\
  &self_cpu_data_.pid, &self_cpu_data_.com, &self_cpu_data_.state, \
  &self_cpu_data_.ppid, &self_cpu_data_.pgrp, &self_cpu_data_.session,\
  &self_cpu_data_.tty_nr, &self_cpu_data_.tpgid, &self_cpu_data_.flags,\
  &self_cpu_data_.minflt, &self_cpu_data_.cminlt, &self_cpu_data_.majflt, &self_cpu_data_.cmajflt, \
  &self_cpu_data_.utime, &self_cpu_data_.stime, &self_cpu_data_.cutime, \
  &self_cpu_data_.cstime, &self_cpu_data_.priority, &self_cpu_data_.nice, &self_cpu_data_.num_threads, \
  &self_cpu_data_.itrealvalue, &self_cpu_data_.starttime, &self_cpu_data_.vsize, \
  &self_cpu_data_.rss, &self_cpu_data_.rsslim, &self_cpu_data_.startcode, \
  &self_cpu_data_.endcode, &self_cpu_data_.startstack, &self_cpu_data_.kstkesp, \
  &self_cpu_data_.kstkeip, &self_cpu_data_.signal, &self_cpu_data_.blocked, \
  &self_cpu_data_.sigignore, &self_cpu_data_.sigcatch, &self_cpu_data_.wchan, \
  &self_cpu_data_.nswap, &self_cpu_data_.cnswap, &self_cpu_data_.exit_signal, \
  &self_cpu_data_.processor, &self_cpu_data_.rt_priority, &self_cpu_data_.policy, \
  &self_cpu_data_.delayacct_blkio_ticks, &self_cpu_data_.guest_time, &self_cpu_data_.cguest_time, \
  &self_cpu_data_.start_data, &self_cpu_data_.end_data, &self_cpu_data_.start_brk, \
  &self_cpu_data_.arg_start, &self_cpu_data_.arg_end, &self_cpu_data_.env_start, \
  &self_cpu_data_.env_end, &self_cpu_data_.exit_code );

  fclose(stat_file);

  if(errno != 0)
  { 
    DEBUG("error in ReadStatsSelfCPU (can't read /proc/self/stat) ");
    return 0;
  }
  else
  {
    return 0;
  }
}

void ResourceUsage::printStatFile(){
  
  printf("%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu\
  %lu %ld %ld %ld %ld %ld %ld %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %lu %lu\
  %lu %lu %lu %lu %d %d %u %u %llu %lu %ld %lu %lu %lu %lu %lu %lu %lu %d \n",\
  self_cpu_data_.pid, self_cpu_data_.com, self_cpu_data_.state, \
  self_cpu_data_.ppid, self_cpu_data_.pgrp, self_cpu_data_.session,\
  self_cpu_data_.tty_nr, self_cpu_data_.tpgid, self_cpu_data_.flags,\
  self_cpu_data_.minflt, self_cpu_data_.cminlt, self_cpu_data_.majflt, self_cpu_data_.cmajflt, \
  self_cpu_data_.utime, self_cpu_data_.stime, self_cpu_data_.cutime, \
  self_cpu_data_.cstime, self_cpu_data_.priority, self_cpu_data_.nice, self_cpu_data_.num_threads, \
  self_cpu_data_.itrealvalue, self_cpu_data_.starttime, self_cpu_data_.vsize, \
  self_cpu_data_.rss, self_cpu_data_.rsslim, self_cpu_data_.startcode, \
  self_cpu_data_.endcode, self_cpu_data_.startstack, self_cpu_data_.kstkesp, \
  self_cpu_data_.kstkeip, self_cpu_data_.signal, self_cpu_data_.blocked, \
  self_cpu_data_.sigignore, self_cpu_data_.sigcatch, self_cpu_data_.wchan, \
  self_cpu_data_.nswap, self_cpu_data_.cnswap, self_cpu_data_.exit_signal, \
  self_cpu_data_.processor, self_cpu_data_.rt_priority, self_cpu_data_.policy, \
  self_cpu_data_.delayacct_blkio_ticks, self_cpu_data_.guest_time, self_cpu_data_.cguest_time, \
  self_cpu_data_.start_data, self_cpu_data_.end_data, self_cpu_data_.start_brk, \
  self_cpu_data_.arg_start, self_cpu_data_.arg_end, self_cpu_data_.env_start, \
  self_cpu_data_.env_end, self_cpu_data_.exit_code );
}

ResourceUsage::~ResourceUsage()
{
  plot_.closeWindow();
}

}  // namespace stats
}  // namespace rhoban_ssl