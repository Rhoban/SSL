#include "timeout_task.h"

namespace rhoban_ssl
{
TimeoutTask::TimeoutTask(double d) : duration(d)
{
}

bool TimeoutTask::runTask()
{
  if (Data::get()->time.now() > duration)
  {
    rhoban_ssl::ExecutionManager::getManager().shutdown();
    return false;
  }
  return true;
}
}  // namespace rhoban_ssl
