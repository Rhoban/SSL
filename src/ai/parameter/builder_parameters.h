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
#include <iostream>
#include <vector>
#include "template_parameter/value_parameter.h"
#include <memory>

namespace rhoban_ssl
{
namespace parameter
{
class BuilderParameters
{
private:
  std::vector<std::shared_ptr<Parameter>> parameters_;

public:
  /**
   * @brief Constructor.
   */
  BuilderParameters();

  /**
   * @brief new_int Create a new int parameters.
   * @param name Name of the parameters.
   * @param comment Comment of the parameters.
   * @param default_value Set the default value (optional - default value : 0).
   * @param writable Is the value is writable (optional - default value : false).
   */
  void new_int(std::string name, std::string comment = "", int default_value = 0, bool writable = false);

  void new_bool(std::string name, std::string comment = "", bool default_value = false, bool writable = false);

  Json::Value getJson();

  void parse(Json::Value tab_json);

  std::shared_ptr<Parameter> getParameterByName(std::string name);

  /**
   * @brief Destructor.
   */
  ~BuilderParameters();
};
}  // namespace parameter
}  // namespace rhoban_ssl
