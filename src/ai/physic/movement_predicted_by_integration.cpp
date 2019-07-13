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

#include "movement_predicted_by_integration.h"
#include <debug.h>

namespace rhoban_ssl
{
Movement* MovementPredictedByIntegration::clone() const
{
  MovementPredictedByIntegration* res = new MovementPredictedByIntegration();
  *res = *this;
  return res;
}

void MovementPredictedByIntegration::print(std::ostream& stream) const
{
  stream << samples_;
};

void MovementPredictedByIntegration::setSample(const MovementSample<ai::Config::samples_history_size>& samples)
{
  assert(samples.isValid());
  samples_ = samples;
}

const MovementSample<ai::Config::samples_history_size>& MovementPredictedByIntegration::getSample() const
{
  return samples_;
}

double MovementPredictedByIntegration::lastTime() const
{
  return samples_.time(0);
}

rhoban_geometry::Point MovementPredictedByIntegration::linearPosition(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  // assert( samples[0].time <= time );
  // double dt=samples.dt(0);
  double dt = time - samples_.time(0);

  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  return (samples_.linearPosition(0) + samples_.linearVelocity(0) * dt  // + samples.linear_acceleration(0) * dt*dt/2.0
          );
}

ContinuousAngle MovementPredictedByIntegration::angularPosition(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  // assert( samples[0].time <= time );
  double dt = time - samples_.time(0);
  return (samples_.angularPosition(0) + (samples_.angularVelocity(0) * dt)  // + (samples.angular_acceleration(0) *
                                                                            // (dt*dt/2.0))
          );
}

Vector2d MovementPredictedByIntegration::linearVelocity(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  // assert( samples[0].time <= time );
  double dt = time - samples_.time(0);
  return samples_.linearVelocity(0) + samples_.linearAcceleration(0) * dt;
}

ContinuousAngle MovementPredictedByIntegration::angularVelocity(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  // assert( samples[0].time <= time );
  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  double dt = time - samples_.time(0);
  return samples_.angularVelocity(0) + samples_.angularAcceleration(0) * dt;
}

Vector2d MovementPredictedByIntegration::linearAcceleration(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  // assert( samples[0].time <= time );
  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  return samples_.linearAcceleration(0);
}

ContinuousAngle MovementPredictedByIntegration::angularAcceleration(double time) const
{
  if (std::fabs(samples_[0].time - time) <= 0.000001)
  {
    time = samples_[0].time;
  }
  // assert( samples[0].time <= time );
  if (!(samples_[0].time <= time))
  {
    DEBUG("WARNING! non monotonous time");
  }
  return samples_.angularAcceleration(0);
}

MovementPredictedByIntegration::~MovementPredictedByIntegration()
{
}

}  // namespace rhoban_ssl
