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

#pragma once

#include <rhoban_utils/util.h>

#include <iostream>
#include <limits>
#include <iomanip>

// TODO: move to rhoban_utils
#define DEBUG(message)                                                                                                 \
  std::cerr << std::setprecision(std::numeric_limits<double>::max_digits10) << "# " << message << " -- "               \
            << rhoban_utils::getBaseName(__FILE__) << ":" << __LINE__ << std::endl

#define DEBUGP(message)                                                                                                \
  if (periodic_debug_is_allowed)                                                                                       \
  {                                                                                                                    \
    std::cerr << std::setprecision(std::numeric_limits<double>::max_digits10) << "# " << message << " -- "             \
              << rhoban_utils::getBaseName(__FILE__) << ":" << __LINE__ << std::endl;                                  \
  }

#define PLOT(message) std::cout << "P " << message << std::endl
