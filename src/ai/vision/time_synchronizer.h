/*
    This file is part of SSL.

    Copyright 2019

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

#include <math/circular_vector.h>
#include "vision_data.h"

namespace rhoban_ssl
{
/**
 * @brief The TimeSynchroniser class computes the time shift with the vision.
 *
 * This class must to be use during the analyse of DetectionPackets in vision.
 */
class TimeSynchronizer
{
public:
  TimeSynchronizer();

  void update(const vision::CameraDetectionFrame& detection_frame);

  void syncTimeShift(double* time_shift_with_vision);

private:
  // todo add to config
  const uint BUFFER_SIZE = 30;

  CircularVector<double> diff_buffer_;
  CircularVector<double> shift_buffer_;

  double computed_time_shift;

private:
  double average(const CircularVector<double>& buffer);
};

}  // namespace rhoban_ssl
