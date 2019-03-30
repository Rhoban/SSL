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

#include "print_collection.h"
#include <sstream>
#include <iostream>
#include <debug.h>

void clear(std::ostringstream& o)
{
  o.str("");
  o.clear();
}

TEST(test_print_collection, print_list)
{
  {
    std::ostringstream s;
    std::list<std::string> l;

    s << l;
    EXPECT_TRUE(s.str() == "<>");
    clear(s);

    l.push_back("voiture");
    s << l;
    EXPECT_TRUE(s.str() == "<voiture, >");
    clear(s);

    l.push_back("maison");
    s << l;
    EXPECT_TRUE(s.str() == "<voiture, maison, >");
    clear(s);
  }
  {
    std::ostringstream s;
    std::list<unsigned int> l;

    s << l;
    EXPECT_TRUE(s.str() == "<>");
    clear(s);

    l.push_back(3);
    s << l;
    EXPECT_TRUE(s.str() == "<3, >");
    clear(s);

    l.push_back(1);
    s << l;
    EXPECT_TRUE(s.str() == "<3, 1, >");
    clear(s);
  }
}

TEST(test_print_collection, print_set)
{
  std::ostringstream s;
  std::set<std::string> set;

  s << set;
  EXPECT_TRUE(s.str() == "{}");
  clear(s);

  set.insert("voiture");
  s << set;
  EXPECT_TRUE(s.str() == "{voiture, }");
  clear(s);

  set.insert("maison");
  s << set;
  EXPECT_TRUE((s.str() == "{voiture, maison, }") or (s.str() == "{maison, voiture, }"));
  clear(s);
}

TEST(test_print_collection, print_map)
{
  std::ostringstream s;
  std::map<std::string, int> map;

  s << map;
  EXPECT_TRUE(s.str() == "{}");
  clear(s);

  map["voiture"] = 1;
  s << map;
  EXPECT_TRUE(s.str() == "{voiture : 1, }");
  clear(s);

  map["maison"] = 2;
  s << map;
  EXPECT_TRUE((s.str() == "{voiture : 1, maison : 2, }") or (s.str() == "{maison : 2, voiture : 1, }"));
  clear(s);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
