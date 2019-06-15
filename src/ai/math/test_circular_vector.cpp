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
#include "circular_vector.h"
#include <sstream>

TEST(test_circular_vector, Constructors)
{
  {
    CircularVector<double> vec(0);
    EXPECT_TRUE(vec.size() == 0);
  }
  {
    CircularVector<double> vec;
    EXPECT_TRUE(vec.size() == 0);
  }
  {
    CircularVector<double> vec(3);
    EXPECT_TRUE(vec.size() == 3);
    for (unsigned int i = 0; i < vec.size(); i++)
    {
      EXPECT_TRUE(vec[i] == 0);
    }
  }
}

TEST(test_circular_vector, insert)
{
  {
    CircularVector<double> vec(3);
    vec.insert(3);
    EXPECT_TRUE(vec[0] == 3);
    EXPECT_TRUE(vec[1] == 0);
    EXPECT_TRUE(vec[2] == 0);

    vec.insert(7);
    EXPECT_TRUE(vec[0] == 7);
    EXPECT_TRUE(vec[1] == 3);
    EXPECT_TRUE(vec[2] == 0);

    vec.insert(13);
    EXPECT_TRUE(vec[0] == 13);
    EXPECT_TRUE(vec[1] == 7);
    EXPECT_TRUE(vec[2] == 3);

    vec.insert(-2);
    EXPECT_TRUE(vec[0] == -2);
    EXPECT_TRUE(vec[1] == 13);
    EXPECT_TRUE(vec[2] == 7);
  }
}

TEST(test_circular_vector, maxSize)
{
  {
    CircularVector<double> vec(42);
    EXPECT_TRUE(vec.size() == 42);
  }
  {
    CircularVector<double> vec(0);
    EXPECT_TRUE(vec.size() == 0);
  }
  {
    CircularVector<double> vec;
    EXPECT_TRUE(vec.size() == 0);
  }
}

TEST(test_circular_vector, size)
{
  {
    CircularVector<double> vec(3);
    DEBUG(vec.numberOfElements());
    EXPECT_TRUE(vec.numberOfElements() == 0);

    vec.insert(-2);
    DEBUG(vec.numberOfElements());
    EXPECT_TRUE(vec.numberOfElements() == 1);

    vec.insert(3);
    DEBUG(vec.numberOfElements());
    EXPECT_TRUE(vec.numberOfElements() == 2);

    vec.insert(4);
    DEBUG(vec.numberOfElements());
    EXPECT_TRUE(vec.numberOfElements() == 3);

    vec.insert(2);
    DEBUG(vec.numberOfElements());
    EXPECT_TRUE(vec.numberOfElements() == 3);
  }
  {
    CircularVector<double> vec;
    EXPECT_TRUE(vec.numberOfElements() == 0);
  }
}

TEST(test_circular_vector, clear)
{
  {
    CircularVector<double> vec(42);
    EXPECT_TRUE(vec.numberOfElements() == 0);

    vec.insert(-2);
    vec.insert(-2);
    vec.insert(-2);
    vec.insert(-2);

    EXPECT_TRUE(vec.numberOfElements() == 4);
    vec.clear();
    EXPECT_TRUE(vec.numberOfElements() == 0);
    EXPECT_TRUE(vec.size() == 42);
  }
}

TEST(test_circular_vector, resize)
{
  {
    CircularVector<double> vec(5);

    vec.insert(1);
    vec.insert(2);
    vec.insert(3);
    vec.insert(4);
    vec.insert(5);
    vec.insert(6);
    vec.insert(7);

    EXPECT_TRUE(vec[0] == 7);
    EXPECT_TRUE(vec[1] == 6);
    EXPECT_TRUE(vec[2] == 5);
    EXPECT_TRUE(vec[3] == 4);
    EXPECT_TRUE(vec[4] == 3);

    EXPECT_TRUE(vec.size() == 5);

    vec.resize(10);

    EXPECT_TRUE(vec.size() == 10);

    EXPECT_TRUE(vec[0] == 7);
    EXPECT_TRUE(vec[1] == 6);
    EXPECT_TRUE(vec[2] == 5);
    EXPECT_TRUE(vec[3] == 4);
    EXPECT_TRUE(vec[4] == 3);

    vec.insert(8);
    vec.insert(9);
    vec.insert(10);

    EXPECT_TRUE(vec[0] == 10);
    EXPECT_TRUE(vec[1] == 9);
    EXPECT_TRUE(vec[2] == 8);
    EXPECT_TRUE(vec[3] == 7);
    EXPECT_TRUE(vec[4] == 6);
    EXPECT_TRUE(vec[5] == 5);
    EXPECT_TRUE(vec[6] == 4);
    EXPECT_TRUE(vec[7] == 3);

    vec.resize(4);

    EXPECT_TRUE(vec.size() == 4);

    EXPECT_TRUE(vec[0] == 10);
    EXPECT_TRUE(vec[1] == 9);
    EXPECT_TRUE(vec[2] == 8);
    EXPECT_TRUE(vec[3] == 7);
  }
}

TEST(test_circular_vector, getters)
{
  {
    CircularVector<double> vec(4);

    vec.insert(1);
    vec.insert(2);
    vec.insert(3);
    vec.insert(4);

    EXPECT_TRUE(vec[0] == 4);
    EXPECT_TRUE(vec[1] == 3);
    EXPECT_TRUE(vec[2] == 2);
    EXPECT_TRUE(vec[3] == 1);

    vec[0] = 14;
    vec[1] = 13;
    vec[2] = 12;
    vec[3] = 11;

    EXPECT_TRUE(vec[0] == 14);
    EXPECT_TRUE(vec[1] == 13);
    EXPECT_TRUE(vec[2] == 12);
    EXPECT_TRUE(vec[3] == 11);
  }
}

TEST(test_circular_vector, stream)
{
  {
    CircularVector<int> vec(4);

    vec.insert(11);
    vec.insert(12);
    vec.insert(13);
    vec.insert(14);

    std::ostringstream s1;
    s1 << vec;

    EXPECT_TRUE("(14, 13, 12, 11)" == s1.str());

    vec[1] = 42;

    s1.str("");
    s1.clear();
    s1 << vec;
    EXPECT_TRUE("(14, 42, 12, 11)" == s1.str());
  }
}

TEST(test_circular_vector, copy)
{
  {
    CircularVector<int> vec(4);

    vec.insert(1);
    vec.insert(2);
    vec.insert(3);
    vec.insert(4);

    CircularVector<int> vec1(vec);

    EXPECT_TRUE(vec1.size() == 4);
    EXPECT_TRUE(vec1[0] == 4);
    EXPECT_TRUE(vec1[1] == 3);
    EXPECT_TRUE(vec1[2] == 2);
    EXPECT_TRUE(vec1[3] == 1);
  }
  {
    CircularVector<int> vec(4);

    vec.insert(1);
    vec.insert(2);
    vec.insert(3);
    vec.insert(4);

    CircularVector<int> vec1;
    vec1 = vec;

    EXPECT_TRUE(vec1.size() == 4);
    EXPECT_TRUE(vec1[0] == 4);
    EXPECT_TRUE(vec1[1] == 3);
    EXPECT_TRUE(vec1[2] == 2);
    EXPECT_TRUE(vec1[3] == 1);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
