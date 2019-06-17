/*
    This file is part of SSL.

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

#include "factory.h"

#include <physic/movement_predicted_by_integration.h>
#include <physic/movement_with_no_prediction.h>
#include <physic/movement_on_new_frame.h>
#include <physic/movement_with_temporal_shift.h>
#include <data.h>

namespace rhoban_ssl
{
namespace physic
{
Movement* Factory::movement()
{
  Movement* movement = nullptr;

  if (ai::Config::enable_movement_with_integration)
  {
    movement = new MovementPredictedByIntegration();
  }
  else
  {
    movement = new MovementWithNoPrediction();
  }
  return new MovementWithTemporalShift(movement);
}

Movement* Factory::robotMovement()
{
  return Factory::movement();
}

Movement* Factory::ballMovement()
{
  return Factory::movement();
}

};  // namespace physic
};  // namespace rhoban_ssl
