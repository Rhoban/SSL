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

#include <manager/manager.h>

namespace rhoban_ssl
{
namespace manager
{
class Manual : Manager
{
private:
public:
  /**
   * @brief Constructor.
   */
  Manual();
  /**
   * @brief Update the manager after each loop.
   */
  void update();
  /**
   * @brief Get all properties of the manager shown in the viewer.
   */
  virtual Json::Value getProperties();
  /**
   * @brief Set the properties send by the viewer.
   */
  virtual void setProperties(Json::Value);

  /**
   * @brief Destructor
   */
  ~Manual();
};
}  // namespace manager
}  // namespace rhoban_ssl
