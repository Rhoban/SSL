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
ValueParameter::ValueParameter(std::string name, std::string comment, bool writable)
  : Parameter(comment, name), writable_(writable)
{
}

ValueParameter::~ValueParameter()
{
}

/**************************************************************************
 *                          Boolean parameter
 **************************************************************************/

BoolParameter::BoolParameter(std::string name, std::string comment, bool value, bool writable)
  : ValueParameter(name, comment, writable), value_(value)
{
}

Json::Value BoolParameter::getJson()
{
  Json::Value json;
  json["name"] = name_;
  json["type"] = "boolean";
  json["value"] = this->value_;
  json["comment"] = this->comment_;
  json["writable"] = this->writable_;
  return json;
}

void BoolParameter::setJson(Json::Value json)
{
  value_ = json["value"].asBool();
}

BoolParameter::~BoolParameter()
{
}

/**************************************************************************
 *                          Integer parameter
 **************************************************************************/

IntParameter::IntParameter(std::string name, std::string comment, int value, bool writable)
  : ValueParameter(name, comment, writable), value_(value)
{
}

Json::Value IntParameter::getJson()
{
  Json::Value json;
  json["name"] = name_;
  json["type"] = "integer";
  json["value"] = this->value_;
  json["comment"] = this->comment_;
  json["writable"] = this->writable_;
  return json;
}

void IntParameter::setJson(Json::Value json)
{
  value_ = json["value"].asInt();
}

IntParameter::~IntParameter()
{
}

/**************************************************************************
 *                          Double parameter
 **************************************************************************/

DoubleParameter::DoubleParameter(std::string name, std::string comment, double value, bool writable)
  : ValueParameter(name, comment, writable), value_(value)
{
}

Json::Value DoubleParameter::getJson()
{
  Json::Value json;
  json["name"] = name_;
  json["type"] = "double";
  json["value"] = this->value_;
  json["comment"] = this->comment_;
  json["writable"] = this->writable_;
  return json;
}

void DoubleParameter::setJson(Json::Value json)
{
  value_ = json["value"].asDouble();
}

DoubleParameter::~DoubleParameter()
{
}

/**************************************************************************
 *                          String parameter
 **************************************************************************/

StringParameter::StringParameter(std::string name, std::string comment, std::string value, bool writable)
  : ValueParameter(name, comment, writable), value_(value)
{
}

Json::Value StringParameter::getJson()
{
  Json::Value json;
  json["name"] = name_;
  json["type"] = "string";
  json["value"] = this->value_;
  json["comment"] = this->comment_;
  json["writable"] = this->writable_;
  return json;
}

void StringParameter::setJson(Json::Value json)
{
  value_ = json["value"].asString();
}

StringParameter::~StringParameter()
{
}
}  // namespace parameter
}  // namespace rhoban_ssl
