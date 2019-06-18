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

#include "value_parameter.h"

namespace rhoban_ssl
{
namespace parameter
{
ValueParameter::ValueParameter(std::string name, std::string comment, bool writable, type_parameters::Type type)
  : Parameter(comment, type), name_(name), writable_(writable)
{
}

ValueParameter::~ValueParameter()
{
}

/**************************************************************************
 *                          Boolean parameter
 **************************************************************************/

/**************************************************************************
 *                          Integer parameter
 **************************************************************************/

IntParameter::IntParameter(std::string name, std::string comment, int value, bool writable)
  : ValueParameter(name, comment, writable, type_parameters::Type::IntParameter), value_(value)
{
}

Json::Value IntParameter::getJson()
{
  Json::Value json;
  json[name_]["type"] = "integer";
  json[name_]["value"] = this->value_;
  json[name_]["comment"] = this->comment_;
  json[name_]["writable"] = this->writable_;
  return json;
}

IntParameter::~IntParameter()
{
}

/**************************************************************************
 *                          Double parameter
 **************************************************************************/

/**************************************************************************
 *                          String parameter
 **************************************************************************/

}  // namespace parameter
}  // namespace rhoban_ssl
