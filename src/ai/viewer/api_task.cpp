/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include <viewer/api_task.h>
#include <viewer/api.h>

namespace rhoban_ssl
{
namespace viewer
{
ApiTask::ApiTask() : Task()
{
  time_last_send_ = GlobalDataSingleThread::singleton_.ai_data_.time;
}

bool ApiTask::runTask()
{
  Api::getApi().readViewerPacket();
  if (GlobalDataSingleThread::singleton_.ai_data_.time - time_last_send_ > 0.150)
  {
    Api::getApi().generateEntityPacket();
    time_last_send_ = GlobalDataSingleThread::singleton_.ai_data_.time;
  }
  return 1;
}

ApiTask::~ApiTask()
{
}
}  // namespace viewer
}  // namespace rhoban_ssl
