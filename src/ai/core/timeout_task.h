#pragma once

#include <execution_manager.h>
#include <data.h>

namespace rhoban_ssl
{
class TimeoutTask : public Task
{
  double duration;

public:
  TimeoutTask(double d);
  virtual bool runTask(void) override;
};
}
