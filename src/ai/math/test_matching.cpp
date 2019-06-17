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
#include "matching.h"
#include <iostream>
#include <core/collection.h>
#include <cmath>

using namespace rhoban_ssl;

struct Man
{
  int value;
  Man(int v) : value(v)
  {
  }
  Man() : value(0.0)
  {
  }
};

std::ostream& operator<<(std::ostream& out, const Man& man)
{
  out << man.value;
  return out;
}

struct Woman
{
  double value;
  Woman(double v) : value(v)
  {
  }
  Woman() : value(0.0)
  {
  }
};

std::ostream& operator<<(std::ostream& out, const Woman& woman)
{
  out << woman.value;
  return out;
}

std::function<double(const Man& juge, const Woman& women)> simple_man_rank = [](const Man& juge, const Woman& women) {
  return women.value;
};

std::function<int(const Woman& juge, const Man& man)> simple_woman_rank = [](const Woman& juge, const Man& man) {
  return man.value;
};

TEST(test_matching, gale_shapley_algorithm__empty_cases)
{
  {
    const std::vector<Man> man_set;
    const std::vector<Woman> woman_set;

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank);

    EXPECT_EQ(matchings.size(), 0);
  }
  {
    const std::vector<Man> man_set = { 3, 4, 1 };
    const std::vector<Woman> woman_set;

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank);

    EXPECT_EQ(matchings.size(), 0);
  }
  {
    const std::vector<Man> man_set;
    const std::vector<Woman> woman_set = { 4.1, 2.0, 1.0 };

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank);

    EXPECT_EQ(matchings.size(), 0);
  }
}

TEST(test_matching, gale_shapley_algorithm__use_cases_with_struct)
{
  {
    const std::vector<Man> man_set = { 1, 2, 4 };
    const std::vector<Woman> woman_set = { 4.1, 2.0, 1.0 };

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank);

    EXPECT_EQ(matchings.size(), 3);
    EXPECT_EQ((map2set(matchings.man_to_women_matchings)),
              (std::set<std::pair<unsigned int, unsigned int> >({ { 0, 2 }, { 1, 1 }, { 2, 0 } })));
  }
}
TEST(test_matching, gale_shapley_algorithm__use_cases_with_double)
{
  {
    const std::vector<double> man_set = { 2.1, 5.0, -1.0 };
    const std::vector<double> woman_set = { 4.1, 1.0, 5.1, 2.0, 1.0 };
    std::function<double(const double& juge, const double& women)> simple_man_rank =
        [](const double& juge, const double& women) { return std::fabs(juge - women); };
    std::function<double(const double& juge, const double& man)> simple_woman_rank =
        [](const double& juge, const double& man) { return std::fabs(juge - man); };

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank);

    EXPECT_EQ(matchings.size(), 3);
    EXPECT_EQ((map2set(matchings.man_to_women_matchings)),
              (std::set<std::pair<unsigned int, unsigned int> >({ { 0, 0 }, { 2, 2 }, { 1, 4 } })));
  }
  {
    const std::vector<double> man_set = { 2.1, 5.0, -1.0, 10.0 };
    const std::vector<double> woman_set = { 1.0, 5.1, 2.0 };
    std::function<double(const double& juge, const double& women)> simple_man_rank =
        [](const double& juge, const double& women) { return std::fabs(juge - women); };
    std::function<double(const double& juge, const double& man)> simple_woman_rank =
        [](const double& juge, const double& man) { return std::fabs(juge - man); };

    matching::Matchings matchings =
        matching::galeShapleyAlgorithm(man_set, woman_set, simple_man_rank, simple_woman_rank, false, false);

    EXPECT_EQ(matchings.size(), 3);
    EXPECT_EQ((map2set(matchings.man_to_women_matchings)),
              (std::set<std::pair<unsigned int, unsigned int> >({ { 2, 0 }, { 1, 1 }, { 0, 2 } })));
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
