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
#include "matrix2d.h"
#include <sstream>

TEST(test_matrix2d, constructor)
{
  {
    Matrix2d m;
    EXPECT_EQ(m(0, 0), 0.0);
    EXPECT_EQ(m(0, 1), 0.0);
    EXPECT_EQ(m(1, 0), 0.0);
    EXPECT_EQ(m(1, 1), 0.0);
  }
  {
    Matrix2d m(1.0, 2.0, 3.0, 4.0);
    EXPECT_EQ(m(0, 0), 1.0);
    EXPECT_EQ(m(0, 1), 2.0);
    EXPECT_EQ(m(1, 0), 3.0);
    EXPECT_EQ(m(1, 1), 4.0);
  }
}

TEST(test_matrix2d, determinant)
{
  {
    Matrix2d m(1, 2, 3, 4);
    EXPECT_EQ(m.det(), 1.0 * 4.0 - 2.0 * 3.0);
  }
}

TEST(test_matrix2d, inverse)
{
  {
    // m = 1 2
    //     3 4
    Matrix2d m(1.0, 2.0, 3.0, 4.0);
    // det = 1*4-2*3 = -2
    // m =  4/-2 -2/-2
    //     -3/-2  1/-2
    Matrix2d i = m.inverse();

    EXPECT_TRUE(std::fabs(i(0, 0) - (-2.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(i(0, 1) - (1.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(i(1, 0) - (3.0 / 2.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(i(1, 1) - (-1.0 / 2.0)) < 0.00001);

    Matrix2d id1 = m * i;
    EXPECT_TRUE(std::fabs(id1(0, 0) - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id1(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id1(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id1(1, 1) - 1.0) < 0.00001);

    Matrix2d id2 = i * m;
    EXPECT_TRUE(std::fabs(id2(0, 0) - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id2(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id2(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(id2(1, 1) - 1.0) < 0.00001);
  }
}

TEST(test_matrix2d, operator_corchet)
{
  {
    const Matrix2d m(2, 3, 4, 5);

    EXPECT_TRUE(std::fabs(m[0][0] - 2.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[0][1] - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[1][0] - 4.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[1][1] - 5.0) < 0.00001);
  }
  {
    Matrix2d m(2, 3, 4, 5);
    m[0][0] = 6;
    m[0][1] = 9;
    m[1][0] = 12;
    m[1][1] = 15;

    EXPECT_TRUE(std::fabs(m[0][0] - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[0][1] - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[1][0] - 12.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m[1][1] - 15.0) < 0.00001);
  }
}

TEST(test_matrix2d, operator_parenthese)
{
  {
    const Matrix2d m(2, 3, 4, 5);

    EXPECT_TRUE(std::fabs(m(0, 0) - 2.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(0, 1) - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(1, 0) - 4.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(1, 1) - 5.0) < 0.00001);
  }
  {
    Matrix2d m(2, 3, 4, 5);
    m(0, 0) = 6;
    m(0, 1) = 9;
    m(1, 0) = 12;
    m(1, 1) = 15;

    EXPECT_TRUE(std::fabs(m(0, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(0, 1) - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(1, 0) - 12.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m(1, 1) - 15.0) < 0.00001);
  }
}

TEST(test_matrix2d, vector2d)
{
  {
    // m = 1 2
    //     3 4
    Matrix2d m(1, 2, 3, 4);

    Vector2d v1(4, 5);
    Vector2d v2 = m * v1;

    EXPECT_TRUE(std::fabs(v2[0] - (4.0 * 1.0 + 5.0 * 2.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(v2[1] - (4.0 * 3.0 + 5.0 * 4.0)) < 0.00001);
  }
}

TEST(test_matrix2d, operator_mult)
{
  {
    const Matrix2d m1(2, 0, 0, 0);
    const Matrix2d m2(3, 0, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 2, 0, 0);
    const Matrix2d m2(3, 0, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 2, 0);
    const Matrix2d m2(3, 0, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 0, 2);
    const Matrix2d m2(3, 0, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }

  {
    const Matrix2d m1(2, 0, 0, 0);
    const Matrix2d m2(0, 3, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 2, 0, 0);
    const Matrix2d m2(0, 3, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 2, 0);
    const Matrix2d m2(0, 3, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 6.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 0, 2);
    const Matrix2d m2(0, 3, 0, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }

  {
    const Matrix2d m1(2, 0, 0, 0);
    const Matrix2d m2(0, 0, 3, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 2, 0, 0);
    const Matrix2d m2(0, 0, 3, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 2, 0);
    const Matrix2d m2(0, 0, 3, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 0, 2);
    const Matrix2d m2(0, 0, 3, 0);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }

  {
    const Matrix2d m1(2, 0, 0, 0);
    const Matrix2d m2(0, 0, 0, 3);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 2, 0, 0);
    const Matrix2d m2(0, 0, 0, 3);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 2, 0);
    const Matrix2d m2(0, 0, 0, 3);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 0.0) < 0.00001);
  }
  {
    const Matrix2d m1(0, 0, 0, 2);
    const Matrix2d m2(0, 0, 0, 3);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 6.0) < 0.00001);
  }

  {
    const Matrix2d m1(1, 2, 3, 4);
    const Matrix2d m2(5, 6, 7, 8);
    Matrix2d m3 = m1 * m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - (1 * 5 + 2 * 7)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - (1 * 6 + 2 * 8)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - (3 * 5 + 4 * 7)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - (3 * 6 + 4 * 8)) < 0.00001);
  }
}

TEST(test_matrix2d, operator_plus)
{
  {
    const Matrix2d m1(1, 2, 3, 4);
    const Matrix2d m2(5, 6, 7, 8);
    Matrix2d m3 = m1 + m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 8.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 10.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 12.0) < 0.00001);
  }
}

TEST(test_matrix2d, sign_plus)
{
  {
    const Matrix2d m1(1, 2, 3, 4);
    Matrix2d m3 = +m1;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 2.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 4.0) < 0.00001);
  }
}

TEST(test_matrix2d, operator_minus)
{
  {
    const Matrix2d m1(10, 20, 30, 40);
    const Matrix2d m2(5, 6, 7, 8);
    Matrix2d m3 = m1 - m2;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 5.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 14.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 23.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 32.0) < 0.00001);
  }
}

TEST(test_matrix2d, sign_minus)
{
  {
    const Matrix2d m1(1, 2, 3, 4);
    Matrix2d m3 = -m1;

    EXPECT_TRUE(std::fabs(m3(0, 0) - (-1.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - (-2.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - (-3.0)) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - (-4.0)) < 0.00001);
  }
}

TEST(test_matrix2d, identity)
{
  {
    Matrix2d m1 = Matrix2d::identity();

    EXPECT_TRUE(std::fabs(m1(0, 0) - 1.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(1, 1) - 1.0) < 0.00001);
  }
}

TEST(test_matrix2d, null)
{
  {
    Matrix2d m1 = Matrix2d::null();

    EXPECT_TRUE(std::fabs(m1(0, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(0, 1) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(1, 0) - 0.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m1(1, 1) - 0.0) < 0.00001);
  }
}

TEST(test_matrix2d, stream)
{
  {
    const Matrix2d m1(1, 2, 3, 4);

    std::ostringstream s1;

    s1 << m1;
    EXPECT_TRUE("[[1, 2], [3, 4]]" == s1.str());
  }
}

TEST(test_matrix2d, operator_scalar_mult)
{
  {
    const Matrix2d m1(1, 2, 3, 4);
    Matrix2d m3 = m1 * 3;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 12.0) < 0.00001);
  }
  {
    const Matrix2d m1(1, 2, 3, 4);
    Matrix2d m3 = 3 * m1;

    EXPECT_TRUE(std::fabs(m3(0, 0) - 3.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(0, 1) - 6.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 0) - 9.0) < 0.00001);
    EXPECT_TRUE(std::fabs(m3(1, 1) - 12.0) < 0.00001);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
