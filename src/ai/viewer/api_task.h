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

#include <execution_manager.h>
#include <data.h>

namespace rhoban_ssl
{
namespace viewer
{
/**
 * @brief API Task to add packet in the queue for the viewer.
 */
class ApiTask : public Task
{
private:
  /**
   * @brief The time when the last packet has put in the API queue.
   */
  double time_last_send_;

public:
  /**
   * @brief Constructor.
   */
  ApiTask();

  /**
   * @brief Run Task and add packet every 150 ms.
   * @return continue Boolean to indicate if the task continue in the next loop of the AI.
   */
  bool runTask() override;

  /**
   * @brief Destructor.
   */
  ~ApiTask();
};
}  // namespace viewer
}  // namespace rhoban_ssl
