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
#include "frame_changement.h"
#include "vector2d.h"

#define EPSILON 0.00001

bool are_equal(const rhoban_geometry::Point& v1, const rhoban_geometry::Point& v2)
{
  return norm(v1 - v2) < EPSILON;
}

bool are_equal(const Vector2d& v1, const Vector2d& v2)
{
  return norm(v1 - v2) < EPSILON;
}

TEST(test_frame_changement, constructor)
{
  FrameChangement fc;

  Vector2d vz(0.0, 0.0);
  Vector2d vi(1.0, 0.0);
  Vector2d vj(0.0, 1.0);

  rhoban_geometry::Point pz(0.0, 0.0);
  rhoban_geometry::Point pi(1.0, 0.0);
  rhoban_geometry::Point pj(0.0, 1.0);

  EXPECT_TRUE(are_equal(vz, fc.toBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.toBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.toBasis(vj)));
  EXPECT_TRUE(are_equal(vz, fc.fromBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.fromBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.fromBasis(vj)));
  EXPECT_TRUE(are_equal(pz, fc.toFrame(pz)));
  EXPECT_TRUE(are_equal(pi, fc.toFrame(pi)));
  EXPECT_TRUE(are_equal(pj, fc.toFrame(pj)));
  EXPECT_TRUE(are_equal(pz, fc.fromFrame(pz)));
  EXPECT_TRUE(are_equal(pi, fc.fromFrame(pi)));
  EXPECT_TRUE(are_equal(pj, fc.fromFrame(pj)));
}

TEST(test_frame_changement, identity)
{
  FrameChangement fc;
  fc.setFrame(rhoban_geometry::Point(0.0, 0.0), Vector2d(1.0, 0.0), Vector2d(0.0, 1.0));

  Vector2d vz(0.0, 0.0);
  Vector2d vi(1.0, 0.0);
  Vector2d vj(0.0, 1.0);

  rhoban_geometry::Point pz(0.0, 0.0);
  rhoban_geometry::Point pi(1.0, 0.0);
  rhoban_geometry::Point pj(0.0, 1.0);

  EXPECT_TRUE(are_equal(vz, fc.toBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.toBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.toBasis(vj)));
  EXPECT_TRUE(are_equal(vz, fc.fromBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.fromBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.fromBasis(vj)));
  EXPECT_TRUE(are_equal(pz, fc.toFrame(pz)));
  EXPECT_TRUE(are_equal(pi, fc.toFrame(pi)));
  EXPECT_TRUE(are_equal(pj, fc.toFrame(pj)));
  EXPECT_TRUE(are_equal(pz, fc.fromFrame(pz)));
  EXPECT_TRUE(are_equal(pi, fc.fromFrame(pi)));
  EXPECT_TRUE(are_equal(pj, fc.fromFrame(pj)));
}

TEST(test_frame_changement, translation)
{
  FrameChangement fc;
  fc.setFrame(rhoban_geometry::Point(1.0, 2.0), Vector2d(1.0, 0.0), Vector2d(0.0, 1.0));

  Vector2d vz(0.0, 0.0);
  Vector2d vi(1.0, 0.0);
  Vector2d vj(0.0, 1.0);

  rhoban_geometry::Point pz(0.0, 0.0);
  rhoban_geometry::Point pi(1.0, 0.0);
  rhoban_geometry::Point pj(0.0, 1.0);

  EXPECT_TRUE(are_equal(vz, fc.toBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.toBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.toBasis(vj)));
  EXPECT_TRUE(are_equal(vz, fc.fromBasis(vz)));
  EXPECT_TRUE(are_equal(vi, fc.fromBasis(vi)));
  EXPECT_TRUE(are_equal(vj, fc.fromBasis(vj)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(-1.0, -2.0), fc.toFrame(pz)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(0.0, -2.0), fc.toFrame(pi)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(-1.0, -1.0), fc.toFrame(pj)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(1.0, 2.0), fc.fromFrame(pz)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(2.0, 2.0), fc.fromFrame(pi)));
  EXPECT_TRUE(are_equal(rhoban_geometry::Point(1.0, 3.0), fc.fromFrame(pj)));
}

TEST(test_frame_changement, rotation)
{
  FrameChangement fc;
  double c = .5;
  double s = std::sqrt(3.0) / 2.0;
  fc.setFrame(rhoban_geometry::Point(0.0, 0.0), Vector2d(c, s), Vector2d(-s, c));

  Vector2d vz(0.0, 0.0);
  Vector2d vi(1.0, 0.0);
  Vector2d vj(0.0, 1.0);

  Vector2d vfi(c, s);
  Vector2d vfj(-s, c);
  Vector2d vti(c, -s);
  Vector2d vtj(s, c);

  rhoban_geometry::Point pz(0.0, 0.0);
  rhoban_geometry::Point pi(1.0, 0.0);
  rhoban_geometry::Point pj(0.0, 1.0);

  rhoban_geometry::Point pfi(c, s);
  rhoban_geometry::Point pfj(-s, c);
  rhoban_geometry::Point pti(c, -s);
  rhoban_geometry::Point ptj(s, c);

  EXPECT_TRUE(are_equal(vz, fc.toBasis(vz)));
  EXPECT_TRUE(are_equal(vti, fc.toBasis(vi)));
  EXPECT_TRUE(are_equal(vtj, fc.toBasis(vj)));
  EXPECT_TRUE(are_equal(vz, fc.fromBasis(vz)));
  EXPECT_TRUE(are_equal(vfi, fc.fromBasis(vi)));
  EXPECT_TRUE(are_equal(vfj, fc.fromBasis(vj)));
  EXPECT_TRUE(are_equal(pz, fc.toFrame(pz)));
  EXPECT_TRUE(are_equal(pti, fc.toFrame(pi)));
  EXPECT_TRUE(are_equal(ptj, fc.toFrame(pj)));
  EXPECT_TRUE(are_equal(pz, fc.fromFrame(pz)));
  EXPECT_TRUE(are_equal(pfi, fc.fromFrame(pi)));
  EXPECT_TRUE(are_equal(pfj, fc.fromFrame(pj)));
}

template <typename VECTOR_CLASS>
double X(const VECTOR_CLASS& v);
template <>
double X<Vector2d>(const Vector2d& v)
{
  return v.getX();
}
template <>
double X<rhoban_geometry::Point>(const rhoban_geometry::Point& p)
{
  return p.getX();
}

template <typename VECTOR_CLASS>
double Y(const VECTOR_CLASS& v);
template <>
double Y<Vector2d>(const Vector2d& v)
{
  return v.getY();
}
template <>
double Y<rhoban_geometry::Point>(const rhoban_geometry::Point& p)
{
  return p.getY();
}

TEST(test_frame_changement, all)
{
  FrameChangement fc;
  double c = .5;
  double s = std::sqrt(3.0) / 2.0;
  fc.setFrame(rhoban_geometry::Point(1.0, 2.0), Vector2d(c, s), Vector2d(-s, c));

  Vector2d vz(0.0, 0.0);
  Vector2d vi(1.0, 0.0);
  Vector2d vj(0.0, 1.0);

  Vector2d vfz(0.0, 0.0);
  Vector2d vfi(c, s);
  Vector2d vfj(-s, c);
  Vector2d vtz(0, 0);
  Vector2d vti(c, -s);
  Vector2d vtj(s, c);

  Vector2d vtr(1.0, 2.0);

  rhoban_geometry::Point pz(0.0, 0.0);
  rhoban_geometry::Point pi(1.0, 0.0);
  rhoban_geometry::Point pj(0.0, 1.0);

  rhoban_geometry::Point pfz(0.0, 0.0);
  rhoban_geometry::Point pfi(c, s);
  rhoban_geometry::Point pfj(-s, c);
  rhoban_geometry::Point ptz(0, 0);
  rhoban_geometry::Point pti(c, -s);
  rhoban_geometry::Point ptj(s, c);

  rhoban_geometry::Point ptr(1.0, 2.0);

  rhoban_geometry::Point f_pfz = ptr;
  rhoban_geometry::Point f_pfi = ptr + pfi;
  rhoban_geometry::Point f_pfj = ptr + pfj;
  rhoban_geometry::Point puz = -ptr;
  rhoban_geometry::Point pui = pi - ptr;
  rhoban_geometry::Point puj = pj - ptr;
  rhoban_geometry::Point f_ptz(X(puz) * c + Y(puz) * s, X(puz) * (-s) + Y(puz) * c);
  rhoban_geometry::Point f_pti(X(pui) * c + Y(pui) * s, X(pui) * (-s) + Y(pui) * c);
  rhoban_geometry::Point f_ptj(X(puj) * c + Y(puj) * s, X(puj) * (-s) + Y(puj) * c);

  EXPECT_TRUE(are_equal(vtz, fc.toBasis(vz)));
  EXPECT_TRUE(are_equal(vti, fc.toBasis(vi)));
  EXPECT_TRUE(are_equal(vtj, fc.toBasis(vj)));
  EXPECT_TRUE(are_equal(vfz, fc.fromBasis(vz)));
  EXPECT_TRUE(are_equal(vfi, fc.fromBasis(vi)));
  EXPECT_TRUE(are_equal(vfj, fc.fromBasis(vj)));
  EXPECT_TRUE(are_equal(f_ptz, fc.toFrame(pz)));
  EXPECT_TRUE(are_equal(f_pti, fc.toFrame(pi)));
  EXPECT_TRUE(are_equal(f_ptj, fc.toFrame(pj)));
  EXPECT_TRUE(are_equal(f_pfz, fc.fromFrame(pz)));
  EXPECT_TRUE(are_equal(f_pfi, fc.fromFrame(pi)));
  EXPECT_TRUE(are_equal(f_pfj, fc.fromFrame(pj)));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
