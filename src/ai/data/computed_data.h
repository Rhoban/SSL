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
#include <math/vector2d.h>
#include <list>
#include "data.h"

namespace rhoban_ssl
{
namespace data
{
/**
 * @brief The ComputedData class
 *
 * This class intent to share computed data accross multiple behaviors/strategies and
 * avoid redundant computation.
 */
class CollisionComputing : public Task
{
public:
  CollisionComputing();

  static std::list<std::pair<int, double> > getCollisions(int robot_id, const Vector2d& linear_velocity);

  // Task interface
public:
  bool runTask();

private:
  void computeTableOfCollisionTimes();
};
}  // namespace data

}  // namespace rhoban_ssl
