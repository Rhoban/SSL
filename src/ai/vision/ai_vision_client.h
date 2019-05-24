/*
  This file is part of SSL.

  Copyright 2018 Grégoire Passault (gregoire.passault@u-bordeaux.fr)
  Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include <VisionClient.h>
#include "vision_data.h"
#include <data.h>
#include "factory.h"
#include "client_config.h"
#include "robot_position_filter.h"
#include "time_synchroniser.h"

namespace rhoban_ssl
{
class SslGeometryPacketAnalyzer : public Task
{
  bool field_done_;
  bool camera_done_;

public:
  SslGeometryPacketAnalyzer();
  virtual bool runTask() override;
};

class DetectionPacketAnalyzer : public Task
{
public:
  virtual bool runTask() override;

private:
  TimeSynchroniser time_synchroniser_;
};

class UpdateRobotInformation : public Task
{
  vision::PartOfTheField part_of_the_field_used_;

public:
  UpdateRobotInformation(vision::PartOfTheField part_of_the_field_used);
  virtual bool runTask() override;
};

class UpdateBallInformation : public Task
{
  vision::PartOfTheField part_of_the_field_used_;

public:
  UpdateBallInformation(vision::PartOfTheField part_of_the_field_used);
  virtual bool runTask() override;
};

}  // namespace rhoban_ssl
