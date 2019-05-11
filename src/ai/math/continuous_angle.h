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

#include <rhoban_utils/angle.h>

class ContinuousAngle
{
private:
  double angle_value_;

public:
  ContinuousAngle();
  ContinuousAngle(double angle);
  //    ContinuousAngle(const Angle& angle);  // Too dangerous, User set_to_nearest
  ContinuousAngle(const ContinuousAngle& angle);

  ContinuousAngle& operator=(double angle);
  ContinuousAngle& operator=(const ContinuousAngle& angle);
  //    ContinuousAngle& operator=( const Angle & angle );

  rhoban_utils::Angle angle() const;
  double value() const;
  ContinuousAngle abs() const;

  ContinuousAngle operator-() const;
  ContinuousAngle operator+() const;

  ContinuousAngle operator-(double angle) const;
  ContinuousAngle operator-(const ContinuousAngle& angle) const;

  ContinuousAngle& operator-=(double angle);
  ContinuousAngle& operator-=(const ContinuousAngle& angle);

  ContinuousAngle operator+(double angle) const;
  ContinuousAngle operator+(const ContinuousAngle& angle) const;

  ContinuousAngle& operator+=(double angle);
  ContinuousAngle& operator+=(const ContinuousAngle& angle);

  ContinuousAngle operator/(double scalar) const;
  ContinuousAngle& operator/=(double scalar);

  ContinuousAngle operator*(double scalar) const;
  ContinuousAngle& operator*=(double scalar);

  double turn() const;
  int nbTurn() const;

  bool operator==(const ContinuousAngle& angle) const;
  bool operator!=(const ContinuousAngle& angle) const;
  bool operator<(const ContinuousAngle& angle) const;
  bool operator<=(const ContinuousAngle& angle) const;
  bool operator>(const ContinuousAngle& angle) const;
  bool operator>=(const ContinuousAngle& angle) const;

  void setToNearest(double angle);
  void setToNearest(const ContinuousAngle& angle);
  void setToNearest(const rhoban_utils::Angle& angle);
};

std::ostream& operator<<(std::ostream& out, const ContinuousAngle& a);
