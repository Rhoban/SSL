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

#include "builder_parameters.h"
#include <debug.h>

namespace rhoban_ssl
{
namespace parameter
{
BuilderParameters::BuilderParameters()
{
}

void BuilderParameters::new_int(std::string name, std::string comment, int value, bool writable)
{
  parameters_.push_back(std::make_shared<IntParameter>(name, comment, value, writable));
}

Json::Value BuilderParameters::getJson()
{
  Json::Value json = Json::arrayValue;
  for (auto it = parameters_.begin(); it != parameters_.end(); ++it)
  {
    json.append((*it)->getJson());
  }

  parameters_.clear();
  return json;
}

BuilderParameters::~BuilderParameters()
{
}

}  // namespace parameter
}  // namespace rhoban_ssl
