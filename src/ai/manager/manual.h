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

#pragma once

#include <manager/manager.h>
#include <viewer/properties.h>

namespace rhoban_ssl
{
namespace manager
{
class Manual : public Manager
{
private:
  bool strategy_was_assigned_;
  bool goal_to_positive_axis_;
  int ally_goalie_id_;
  int opponent_goalie_id_;
  viewer::PropertiesFactory properties_factory;

public:
  /**
   * @brief Constructor.
   */
  Manual(std::string name);
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
