/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com) (Refacto)

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
#include "vector2d.h"

TEST(test_vector2d, vectorial_product)
{
  {
    Vector2d v1(0.0, 0.0);
    Vector2d v2(0.0, 0.0);
    EXPECT_EQ(vectorialProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(1.0, 2.0);
    Vector2d v2(0.0, 0.0);
    EXPECT_EQ(vectorialProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(0.0, 0.0);
    Vector2d v2(3.0, 4.0);
    EXPECT_EQ(vectorialProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(2.0, 3.0);
    Vector2d v2(4.0, 5.0);
    EXPECT_EQ(vectorialProduct(v1, v2), 2.0 * 5.0 - 4.0 * 3.0);
  }
}

TEST(test_vector2d, scalar_product)
{
  {
    Vector2d v1(0.0, 0.0);
    Vector2d v2(0.0, 0.0);
    EXPECT_EQ(scalarProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(1.0, 2.0);
    Vector2d v2(0.0, 0.0);
    EXPECT_EQ(scalarProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(0.0, 0.0);
    Vector2d v2(3.0, 4.0);
    EXPECT_EQ(scalarProduct(v1, v2), 0.0);
  }
  {
    Vector2d v1(2.0, 3.0);
    Vector2d v2(4.0, 5.0);
    EXPECT_EQ(scalarProduct(v1, v2), 2.0 * 4.0 + 3.0 * 5.0);
  }
}

TEST(test_vector2d, norm)
{
  {
    Vector2d v(0.0, 0.0);
    EXPECT_EQ(norm(v), 0.0);
  }
  {
    Vector2d v(1.0, 0.0);
    EXPECT_EQ(norm(v), 1.0);
  }
  {
    Vector2d v(0.0, 1.0);
    EXPECT_EQ(norm(v), 1.0);
  }
  {
    Vector2d v(3.0, 4.0);
    EXPECT_EQ(norm(v), 5.0);
  }
  {
    Vector2d v(4.0, 3.0);
    EXPECT_EQ(norm(v), 5.0);
  }
  {
    Vector2d v(0.0, 0.0);
    EXPECT_TRUE(std::fabs(v.norm()) < 0.0001);
  }
  {
    Vector2d v(2.0, 0.0);
    EXPECT_TRUE(std::fabs(v.norm() - 2.0) < 0.0001);
  }
  {
    Vector2d v(0.0, 2.0);
    EXPECT_TRUE(std::fabs(v.norm() - 2.0) < 0.0001);
  }
  {
    Vector2d v(1.0, 1.0);
    EXPECT_TRUE(std::fabs(v.norm() - std::sqrt(2.0)) < 0.0001);
    EXPECT_TRUE(std::fabs(v.norm() - norm(v)) < 0.0001);
    EXPECT_TRUE(std::fabs(v.norm() - norm_2(v)) < 0.0001);
  }
}

TEST(test_vector2d, norm_square)
{
  {
    Vector2d v(0.0, 0.0);
    EXPECT_EQ(v.normSquare(), 0.0);
    EXPECT_EQ(normSquare(v), 0.0);
  }
  {
    Vector2d v(1.0, 0.0);
    EXPECT_EQ(normSquare(v), 1.0);
  }
  {
    Vector2d v(0.0, 1.0);
    EXPECT_EQ(v.normSquare(), 1.0);
    EXPECT_EQ(normSquare(v), 1.0);
  }
  {
    Vector2d v(3.0, 4.0);
    EXPECT_EQ(v.normSquare(), 25.0);
    EXPECT_EQ(normSquare(v), 25.0);
  }
  {
    Vector2d v(4.0, 3.0);
    EXPECT_EQ(v.normSquare(), 25.0);
    EXPECT_EQ(normSquare(v), 25.0);
  }
  {
    Vector2d v(0.0, 0.0);
    EXPECT_TRUE(std::fabs(v.normSquare()) < 0.0001);
    EXPECT_TRUE(std::fabs(normSquare(v)) < 0.0001);
  }
  {
    Vector2d v(2.0, 0.0);
    EXPECT_TRUE(std::fabs(v.normSquare() - 4.0) < 0.0001);
    EXPECT_TRUE(std::fabs(normSquare(v) - 4.0) < 0.0001);
  }
  {
    Vector2d v(0.0, 2.0);
    EXPECT_TRUE(std::fabs(v.normSquare() - 4.0) < 0.0001);
    EXPECT_TRUE(std::fabs(normSquare(v) - 4.0) < 0.0001);
  }
  {
    Vector2d v(1.0, 1.0);
    EXPECT_TRUE(std::fabs(v.normSquare() - 2.0) < 0.0001);
    EXPECT_TRUE(std::fabs(normSquare(v) - 2.0) < 0.0001);
  }
}

TEST(test_vector2d, normalized)
{
  {
    Vector2d v(1.0, 0.0);
    EXPECT_EQ(normalized(v), Vector2d(1.0, 0.0));
  }
  {
    Vector2d v(2.0, 0.0);
    EXPECT_EQ(normalized(v), Vector2d(1.0, 0.0));
  }
  {
    Vector2d v(0.0, 1.0);
    EXPECT_EQ(normalized(v), Vector2d(0.0, 1.0));
  }
  {
    Vector2d v(0.0, 2.0);
    EXPECT_EQ(normalized(v), Vector2d(0.0, 1.0));
  }
  {
    Vector2d v(3.0, 4.0);
    EXPECT_EQ(normalized(v), Vector2d(3.0 / 5.0, 4.0 / 5.0));
  }
}

TEST(test_vector2d, vector2point)
{
  {
    Vector2d v(0.0, 0.0);
    rhoban_geometry::Point p = vector2point(v);
    EXPECT_EQ(p.getX(), 0.0);
    EXPECT_EQ(p.getY(), 0.0);
  }
  {
    Vector2d v(12.0, 42.0);
    rhoban_geometry::Point p = vector2point(v);
    EXPECT_EQ(p.getX(), v[0]);
    EXPECT_EQ(p.getY(), v[1]);
  }
}

TEST(test_vector2d, point2vector)
{
  {
    rhoban_geometry::Point p(0.0, 0.0);
    Vector2d v = point2vector(p);
    EXPECT_EQ(v[0], 0.0);
    EXPECT_EQ(v[1], 0.0);
  }
  {
    rhoban_geometry::Point p(12.0, 42.0);
    Vector2d v = point2vector(p);
    EXPECT_EQ(p.getX(), v[0]);
    EXPECT_EQ(p.getY(), v[1]);
  }
}

TEST(test_vector2d, operator_crochet)
{
  {
    const Vector2d v(3.0, 42.0);
    EXPECT_TRUE(std::fabs(v[0] - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v[1] - 42.0) < 0.00001);
  }
  {
    Vector2d v(3.0, 42.0);
    v[0] = 9.0;
    v[1] = 26.0;
    EXPECT_TRUE(std::fabs(v[0] - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v[1] - 26.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_plus)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2(4.0, 5.0);
    Vector2d v3 = v1 + v2;
    EXPECT_TRUE(std::fabs(v3[0] - 7.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v3[1] - 47.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_minus)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2(4.0, 5.0);
    Vector2d v3 = v1 - v2;
    EXPECT_TRUE(std::fabs(v3[0] - (-1.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v3[1] - 37.0) < 0.00001);
  }
}

TEST(test_vector2d, sign_plus)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2 = +v1;
    EXPECT_TRUE(std::fabs(v2[0] - (3.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 42.0) < 0.00001);
  }
}

TEST(test_vector2d, sign_minus)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2 = -v1;
    EXPECT_TRUE(std::fabs(v2[0] - (-3.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - (-42.0)) < 0.00001);
  }
}

TEST(test_vector2d, operator_plus_eq)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2(4.0, 5.0);
    v2 += v1;
    EXPECT_TRUE(std::fabs(v2[0] - 7.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 47.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_minus_eq)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2(4.0, 5.0);
    v2 -= v1;
    EXPECT_TRUE(std::fabs(v2[0] - (1.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - (-37.0)) < 0.00001);
  }
}

TEST(test_vector2d, operator_eq)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2;
    v2 = v1;
    EXPECT_TRUE(std::fabs(v2[0] - (3.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - (42.0)) < 0.00001);
  }
}

TEST(test_vector2d, operator_mult)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2 = 3 * v1;
    Vector2d v3 = v1 * 3;
    EXPECT_TRUE(std::fabs(v2[0] - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 126.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v3[0] - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v3[1] - 126.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_mult_eq)
{
  {
    Vector2d v1(3.0, 42.0);
    v1 *= 3;
    EXPECT_TRUE(std::fabs(v1[0] - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v1[1] - 126.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_div)
{
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2 = v1 / 3;
    EXPECT_TRUE(std::fabs(v2[0] - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 14.0) < 0.00001);
  }
}

TEST(test_vector2d, operator_div_eq)
{
  {
    Vector2d v1(3.0, 42.0);
    v1 /= 3;
    EXPECT_TRUE(std::fabs(v1[0] - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v1[1] - 14.0) < 0.00001);
  }
}

TEST(test_vector2d, perpendicular)
{
  {
    Vector2d v1(1.0, 0.0);
    Vector2d v2 = v1.perpendicular();
    EXPECT_TRUE(std::fabs(v2[0] - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 1.0) < 0.00001);
  }
  {
    Vector2d v1(0.0, 1.0);
    Vector2d v2 = v1.perpendicular();
    EXPECT_TRUE(std::fabs(v2[0] - (-1.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 0.0) < 0.00001);
  }
  {
    Vector2d v1(3.0, 42.0);
    Vector2d v2 = v1.perpendicular();
    EXPECT_TRUE(std::fabs(v2[0] - (-42)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - 3.0) < 0.00001);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
