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
#include "parameter.h"

namespace rhoban_ssl
{
namespace parameter
{
class ValueParameter : public Parameter
{
protected:
  /**
   * @brief Indicate if the user can change the parameter.
   */
  bool writable_;

public:
  /**
   * @brief Constructor of the class (Default to writable)
   * @param name Name of the parameter.
   * @param comment Detail of the parameter.
   * @param writable Indicate if the user can change the parameter.
   */
  ValueParameter(std::string name, std::string comment, bool writable, type_parameters::Type type);


  /**
   * @brief Destructor.
   */
  virtual ~ValueParameter();
};

/**************************************************************************
 *                          Boolean parameter
 **************************************************************************/

class BoolParameter : public ValueParameter
{
private:
  /**
   * @brief The value of the parameter.
   */
  bool value_;

public:
  /**
   * @brief Constructor of the class
   * @param name Name of the parameter.
   * @param comment Detail of the parameter.
   * @param writable Indicate if the user can change the parameter.
   */
  BoolParameter(std::string name, std::string comment, bool value, bool writable);
  virtual Json::Value getJson();
  virtual void setJson(Json::Value json);
  virtual ~BoolParameter();
};

/**************************************************************************
 *                          Integer parameter
 **************************************************************************/

class IntParameter : public ValueParameter
{
private:
  /**
   * @brief The value of the parameter.
   */
  int value_;

public:
  /**
   * @brief Constructor of the class
   * @param name Name of the parameter.
   * @param comment Detail of the parameter.
   * @param writable Indicate if the user can change the parameter.
   */
  IntParameter(std::string name, std::string comment, int value, bool writable);
  virtual Json::Value getJson();
  virtual void setJson(Json::Value json);
  virtual ~IntParameter();
};

}  // namespace parameter
}  // namespace rhoban_ssl
