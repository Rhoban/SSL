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

#include "robot_behavior.h"

TEST(test_robot_behavior, constructors)
{
  {
    rhoban_ssl::Control p;
    EXPECT_TRUE(!p.kick);
    EXPECT_TRUE(p.active);
    EXPECT_TRUE(!p.ignore);
    //        EXPECT_TRUE( p == PidControl() );
  }

  {
    bool cases[8][3] = {
      { false, false, false }, { false, false, true }, { false, true, false }, { false, true, true },
      { true, false, false },  { true, false, true },  { true, true, false },  { true, true, true }
    };
    for (int i = 0; i < 8; i++)
    {
      rhoban_ssl::Control p(cases[i][0], cases[i][1], cases[i][2]);
      EXPECT_TRUE(p.kick == cases[i][0]);
      EXPECT_TRUE(p.active == cases[i][1]);
      EXPECT_TRUE(p.ignore == cases[i][2]);
    }
  }
}

TEST(test_robot_behavior, make_desactivated)
{
  {
    rhoban_ssl::Control c = rhoban_ssl::Control::make_desactivated();
    EXPECT_TRUE(c.kick == false);
    EXPECT_TRUE(c.active == false);
    EXPECT_TRUE(c.ignore == false);
  }
}

TEST(test_robot_behavior, make_ignored)
{
  {
    rhoban_ssl::Control c = rhoban_ssl::Control::make_ignored();
    EXPECT_TRUE(c.kick == false);
    EXPECT_TRUE(c.active == false);
    EXPECT_TRUE(c.ignore == true);
  }
}

TEST(test_robot_behavior, make_null)
{
  {
    rhoban_ssl::Control c = rhoban_ssl::Control::make_null();
    EXPECT_TRUE(c.kick == false);
    EXPECT_TRUE(c.active == true);
    EXPECT_TRUE(c.ignore == false);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
