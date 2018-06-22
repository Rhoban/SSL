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

#ifndef __MOVEMENT_PREDICTION_BY_INTEGRATION_H__ 
#define __MOVEMENT_PREDICTION_BY_INTEGRATION_H__ 

#include <physic/Movement.h>
#include <AiData.h>

namespace RhobanSSL {

class Movement_predicted_by_integration : public Movement {
private:
  MovementSample samples;
  Ai::AiData & ai_data;
  void check();

public:
  virtual Movement * clone() const;

  virtual void set_sample( const MovementSample & samples );
  virtual const MovementSample & get_sample() const;

  virtual double last_time() const;

  virtual rhoban_geometry::Point linear_position( double time ) const;
  virtual ContinuousAngle angular_position( double time ) const;

  virtual Vector2d linear_velocity( double time ) const;
  virtual ContinuousAngle angular_velocity( double time ) const;

  virtual Vector2d linear_acceleration( double time ) const;
  virtual ContinuousAngle angular_acceleration( double time ) const;
  virtual double get_dt(double time) const;
  virtual void print(std::ostream& stream) const;
  Movement_predicted_by_integration(Ai::AiData & ai_data);
  Movement_predicted_by_integration(const Movement_predicted_by_integration &tocopy);
  virtual ~Movement_predicted_by_integration();
};

}

#endif
