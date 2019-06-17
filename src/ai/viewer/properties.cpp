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

#include "properties.h"

namespace rhoban_ssl
{
namespace viewer
{
PropertiesFactory::PropertiesFactory()
{
}

void PropertiesFactory::addSetValue(std::string name, std::string default_value)
{
  Json::Value property;

  property["type"] = "Input";
  property["type_value"] = "String";

  property["name"] = name;
  property["value"] = default_value;

  json.append(property);
}

void PropertiesFactory::addSetValue(std::string name, int default_value)
{
  Json::Value property;

  property["type"] = "Input";
  property["type_value"] = "Integer";

  property["name"] = name;
  property["value"] = default_value;

  json.append(property);
}

void PropertiesFactory::addSetValue(std::string name, double default_value)
{
  Json::Value property;

  property["type"] = "Input";
  property["type_value"] = "Double";

  property["name"] = name;
  property["value"] = default_value;

  json.append(property);
}

void PropertiesFactory::addRangeValue(std::string name, int min, int max, int step, int default_value)
{
  Json::Value property;

  property["type"] = "Range";
  property["type_value"] = "Integer";

  property["name"] = name;
  property["min"] = min;
  property["max"] = max;
  property["step"] = step;
  property["value"] = default_value;

  json.append(property);
}

void PropertiesFactory::addRangeValue(std::string name, double min, double max, double step, double default_value)
{
  Json::Value property;

  property["type"] = "Range";
  property["type_value"] = "Double";

  property["name"] = name;
  property["min"] = min;
  property["max"] = max;
  property["step"] = step;
  property["value"] = default_value;

  json.append(property);
}

void PropertiesFactory::clear()
{
  json.clear();
}

Json::Value& PropertiesFactory::getJson()
{
  return json;
}

PropertiesFactory::~PropertiesFactory()
{
}

}  // namespace viewer
}  // namespace rhoban_ssl
