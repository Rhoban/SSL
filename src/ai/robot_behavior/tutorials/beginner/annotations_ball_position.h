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

#include <robot_behavior/robot_behavior.h>
#include <robot_behavior/factory.h>
#include <parameter/builder_parameters.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace beginner
{
/** Tutorial class to show how to move a robot in the side corner. */
class AnnotationsBallPosition : public RobotBehavior
{
private:
  rhoban_ssl::annotations::Annotations annotations_;
  parameter::BuilderParameters builder_parameter_;
  bool show_annotation_;

public:
  AnnotationsBallPosition();

  virtual void update(double time, const data::Robot& robot, const data::Ball& ball);

  virtual Control control() const;

  virtual rhoban_ssl::annotations::Annotations getAnnotations() const;

  virtual Json::Value getParameters();

  virtual void setParameters(Json::Value);

  virtual ~AnnotationsBallPosition();
};

};  // namespace beginner
};  // namespace robot_behavior
};  // namespace rhoban_ssl
