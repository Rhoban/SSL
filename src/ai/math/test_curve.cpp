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

#include <gtest/gtest.h>

#include <debug.h>
#include "curve.h"

#include <iostream>
#include <cmath>

bool eq(double u, double v, double err)
{
  return std::fabs(u - v) <= err;
}

double error_renormalisation(double t, const RenormalizedCurve& curve, std::function<double(double)>& error_curve,
                             double dt)
{
  return (error_curve(curve.inverseOfArcLength(curve.positionConsign(t))) +
          error_curve(curve.inverseOfArcLength(curve.positionConsign(t + dt)))) /
         dt;
};

TEST(test_curve, curves)
{
  {
    double length = 2.0;

    // length = \int_0^time_max 1 = time_max
    double time_max = length;
    double dt = time_max / 100;
    double dt_micro = dt / 100;
    double du = dt_micro / time_max;
    double erreur = dt / 10;

    RenormalizedCurve curve([&](double u) { return Vector2d(length * u, 0.0); }, du,
                            [&](double t) { return time_max / length; }, dt_micro, dt_micro / 100);
    for (double u = 0.0; u <= 1.0; u = u + dt)
    {
      EXPECT_TRUE(curve.originalCurve(u) == Vector2d(2 * u, 0));
    }

    EXPECT_TRUE(eq(length, curve.size(), erreur));

    // As velocity is 1, then length is equal to time to walk on the
    // complete curve.
    EXPECT_TRUE(eq(length, curve.maxTime(), erreur));
    // On this example the ration between time en distance is 2.
    EXPECT_TRUE(eq(length, curve.time(length - erreur), 2 * erreur));

    RenormalizedCurve::TimeCurve time_it = curve.timeIterator();
    for (double d = 0.0; d <= length; d = d + dt)
    {
      EXPECT_TRUE(eq(d, time_it(d), erreur));
    }

    RenormalizedCurve::PositionConsign position_it = curve.positionConsignIterator();
    for (double t = 0.0; t < 2 * length; t = t + dt)
    {
      EXPECT_TRUE(eq(t, position_it(t), erreur));
    }

    Curve2d::Length length_it = curve.lengthIterator();
    for (double u = 0.0; u < 1.0; u = u + dt)
    {
      EXPECT_TRUE(eq(length * u, length_it(u), erreur));
    }

    Curve2d::InverseOfLength inverse_legnth_it = curve.InverseOfLengthIterator();
    for (double l = 0.0; l <= length; l = l + dt)
    {
      EXPECT_TRUE(eq(l / length, inverse_legnth_it(l), erreur));
    }

    std::function<double(double)> error_curve = [&](double u) { return 2 * dt_micro; };
    RenormalizedCurve::CurveIterator curve_t_it = curve.curveIterator();
    RenormalizedCurve::CurveIterator curve_t_dt_it = curve.curveIterator();
    for (double t = 0.0; t < curve.maxTime() - dt; t = t + dt)
    {
      EXPECT_TRUE(eq(1.0, (norm_2(curve_t_dt_it(t + dt) - curve_t_it(t))) / dt,
                     error_renormalisation(t, curve, error_curve, dt) + dt_micro / 10.0));
    }

    EXPECT_TRUE(eq(norm_2(curve(curve.maxTime() + dt) - curve(curve.maxTime())), 0, erreur));
    EXPECT_TRUE(curve(curve.maxTime() + dt) == curve(curve.maxTime() + 2 * dt));
    EXPECT_TRUE(curve(curve.maxTime() + 3 * dt) == curve(curve.maxTime() + dt));
  }

  {
    // Length = \int_0^1 sqrt( 4*u**2 + 4 ) =
    //        = 2 * ( 1/2*sqrt(x^2 + 1)*x + 1/2*arcsinh(x) )
    // length = sqrt(2) + arcsinh(1) - 0 - 0 = 2.2955871
    double length = std::sqrt(2) + std::asinh(1);
    // length = \int_0^time_max t+0.1
    // length = [ t^2/2 + 0.1 *t ]_0^time_max
    //
    double time_max = -0.1 + sqrt(.1 * .1 + 2 * length);
    double dt = time_max / 10;
    double dt_micro = dt / 100;
    double du = dt_micro;  // dt_micro/time_max;
    double erreur = dt / 10;

    RenormalizedCurve curve([](double u) { return Vector2d(u * u, 2 * u); }, dt_micro, [](double t) { return t + .1; },
                            dt_micro, dt_micro / 100);
    EXPECT_TRUE(eq(length, curve.size(), erreur));
    EXPECT_TRUE(eq(time_max, curve.getTimeMax(), erreur));
    std::function<double(double)> error_curve = [&](double u) {
      return ((std::sqrt(u + du) + std::asinh(u + du) - std::sqrt(u) + std::asinh(u)) -
              (sqrt(std::pow(2 * du + du * du, 2) + std::pow(2 * du, 2))));
      // return 2*du*std::sqrt( std::pow( u+du/2.0, 2 ) + 1 );
    };
    RenormalizedCurve::CurveIterator curve_t_it = curve.curveIterator();
    RenormalizedCurve::CurveIterator curve_t_dt_it = curve.curveIterator();
    for (double t = 0.0; t < curve.maxTime() - dt; t = t + dt)
    {
      /* TODO change du to onther valuer */
      /*
      DEBUG("########################");
      DEBUG("size : " << curve.size() );
      DEBUG("length : " << length );
      DEBUG("time_max : " << time_max );
      DEBUG("time_max 1  : " << curve.get_time_max() );
      DEBUG("t+.1 : " << t+.1 );
      DEBUG( norm_2( curve_t_dt_it(t+dt) - curve_t_it(t) )/dt );
      DEBUG(
          "error_ren : " <<
              error_renormalisation(t, curve, error_curve, dt) + du/10.0
      );
      */
      EXPECT_TRUE(eq(t + .1, norm_2(curve_t_dt_it(t + dt) - curve_t_it(t)) / dt,
                     error_renormalisation(t, curve, error_curve, dt)  //+ du/10.0
                     ));
      // DEBUG("");
    }
    EXPECT_TRUE(curve(curve.maxTime() + dt) == curve(curve.maxTime()));
    EXPECT_TRUE(curve(curve.maxTime() + 3 * dt) == curve(curve.maxTime()));
  }
}

TEST(test_curve, velocityconsign)
{
  {
    double erreur = 0.0000001;
    double distance = 4.0;
    double max_acceleration = 3.0;
    double max_velocity = 2.0;
    ContinuousVelocityConsign consign(distance, max_velocity, max_acceleration);
    EXPECT_TRUE(consign(-0.1) == 0.0);
    EXPECT_TRUE(consign(0) == 0.0);
    EXPECT_TRUE(eq(consign(consign.time_of_acceleration() / 2.0), max_velocity / 2.0, erreur));
    EXPECT_TRUE(eq(consign(consign.time_of_acceleration()), max_velocity, erreur));
    EXPECT_TRUE(eq(consign(consign.time_of_deplacement() - consign.time_of_acceleration()), max_velocity, erreur));
    EXPECT_TRUE(
        eq(consign(consign.time_of_deplacement() - consign.time_of_acceleration() / 2.0), max_velocity / 2.0, erreur));
    EXPECT_TRUE(consign(consign.time_of_deplacement()) == 0.0);
    EXPECT_TRUE(consign(consign.time_of_deplacement() + .01) == 0.0);
    double len = 0.0;
    double step = .00001;
    for (double t = 0.0; t < consign.time_of_deplacement(); t += step)
    {
      len += consign(t) * step;
    }
    EXPECT_TRUE(eq(distance, len, .0001));
  }
}

TEST(test_curve, empty_curves)
{
  {
    double dt = 0.01;
    double dt_micro = dt / 100;

    Vector2d position(3.0, 7.0);
    RenormalizedCurve curve([&](double u) { return position; }, dt_micro, [](double t) { return 1.0; }, dt_micro,
                            dt_micro / 100);

    EXPECT_TRUE(0.0 == curve.size());
    EXPECT_TRUE(0.0 == curve.maxTime());
    EXPECT_TRUE(position == curve(0.0));
    EXPECT_TRUE(position == curve(0.4));
  }
}

struct Rotation
{
  double orientation;

  double operator()(double u) const
  {
    return 90.0 * u + orientation;
  };
};

struct Translation
{
  Vector2d position;

  Vector2d operator()(double u) const
  {
    return position + Vector2d(u, u * u);
  };

  Translation() : position(0.0, 0.0){};
};

struct fct_wrapper
{
  std::function<double(double u)> rotation;

  fct_wrapper(const std::function<double(double u)>& rotation) : rotation(rotation){};

  Vector2d operator()(double t)
  {
    return Vector2d(rotation(t), 0.0);
  };
};

TEST(test_curve, use_cases)
{
  {
    Rotation rot;
    rot.orientation = 90.0;
    fct_wrapper fct(rot);
    Curve2d curve(fct, 0.0001);
    EXPECT_TRUE(curve.size() == 90);
  }

  {
    double dt = 0.01;
    double dt_micro = dt / 100;

    RenormalizedCurve curve([](double u) { return Vector2d(90.0 * u, 0.0); }, dt_micro, [](double t) { return 10.0; },
                            dt_micro, dt_micro / 100);
    EXPECT_TRUE(eq(curve.maxTime(), 9, 0.01));
  }
  {
    double dt = 0.01;
    double dt_micro = dt / 100;

    Curve2d crv([](double u) { return Vector2d(90.0 * u, 0.0); }, dt_micro);
    EXPECT_TRUE(crv.size() == 90);
    double max_acceleration = 300.0;
    double max_velocity = 90.0;
    ContinuousVelocityConsign consign(crv.size(), max_velocity, max_acceleration);

    RenormalizedCurve curve(crv, consign, dt_micro, dt_micro / 100);
  }
  {
    double dt = 0.001;
    double dt_micro = 0.0001;

    Translation trans;
    Curve2d crv(trans, dt_micro);
    double max_acceleration = 20.0;
    double max_velocity = 1.0;
    ContinuousVelocityConsign consign(crv.size(), max_velocity, max_acceleration);
    RenormalizedCurve curve(crv, consign, dt_micro, dt_micro / 100);

    RenormalizedCurve::CurveIterator curve_t_it = curve.curveIterator();
    RenormalizedCurve::CurveIterator curve_t_dt_it = curve.curveIterator();
    for (double t = 0; t < curve.maxTime() - dt; t += dt)
    {
      Vector2d xt = curve_t_it(t);
      Vector2d xdt = curve_t_dt_it(t + dt);
      EXPECT_TRUE(xdt[0] >= xt[0]);
      EXPECT_TRUE(xdt[1] >= xt[1]);
    }
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
