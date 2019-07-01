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

#include "task_example.h"
#include <debug.h>

namespace rhoban_ssl
{
TaskExample::TaskExample()
{
  compter_ = 0;
}

bool TaskExample::runTask()
{
  compter_++;
  if (compter_ == 1000)
  {
    DEBUG("compter_ == 1000");
    return false;
  }
  return true;
}

TaskExample::~TaskExample()
{
}

}  // namespace rhoban_ssl