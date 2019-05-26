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
#include <json/json.h>
#include <list>

namespace rhoban_ssl
{
namespace viewer
{
class PropertiesFactory
{
private:
  Json::Value json;

public:
  PropertiesFactory();

  void addSetValue(std::string name, std::string default_value);

  void addSetValue(std::string name, int default_value);

  void addSetValue(std::string name, double default_value);

  void addRangeValue(std::string name, int min, int max, int step, int default_value);

  void addRangeValue(std::string name, double min, double max, double step, double default_value);

  // void addList(std::string name, std::list<std::string> string_list);

  // void addList(std::string name, std::list<int> string_list);

  // void addList(std::string name, std::list<double> string_list);

  void clear();

  Json::Value& getJson();

  ~PropertiesFactory();
};
}  // namespace viewer
}  // namespace rhoban_ssl
