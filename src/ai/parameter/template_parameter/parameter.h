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
#include <json/json.h>

namespace rhoban_ssl
{
namespace type_parameters
{
enum Type
{
  NoneParameter,
  IntParameter
};
}

namespace parameter
{
class Parameter
{
protected:
  type_parameters::Type type_;
  std::string comment_;
  std::string name_;
public:
  Parameter(std::string comment, type_parameters::Type type, std::string name);
  /**
   * @brief Obtain the json of the parameters.
   * @return
   */

  virtual std::string getName();

  virtual Json::Value getJson() = 0;

  virtual void setJson(Json::Value json) = 0;
  /**
   * @brief Destructor of the class.
   */
  virtual ~Parameter();
};
}  // namespace parameter
}  // namespace rhoban_ssl
