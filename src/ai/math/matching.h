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

#ifndef __MATH__STABLE_MATCHING__H__
#define __MATH__STABLE_MATCHING__H__

#include <vector>
#include <algorithm>
#include <utility>
#include <functional>
#include <map>
#include <list>

#include <debug.h>
#include <core/print_collection.h>

namespace RhobanSSL
{
namespace matching
{
struct Matchings
{
  std::map<unsigned int, unsigned int> women_to_man_matchings;  // (Women, Men)
  std::map<unsigned int, unsigned int> man_to_women_matchings;  // (Women, Men)
  std::list<unsigned int> unaffected_man;
  std::list<unsigned int> unaffected_woman;

  int size() const
  {
    return women_to_man_matchings.size();
  };
};

/*
 * This is an implementation of the gale-shaplery algorithm for computing stable matching
 *
 * Cf article :
 *   D. Gale et L. S. Shapley,
 *   « College Admissions and the Stability of Marriage »,
 *   in Amer. Math. Month., vol. 69, 1962, p. 9-14.
 *
 */
template <typename MAN, typename WOMAN, typename MAN_RANK, typename WOMAN_RANK>
Matchings gale_shapley_algorithm(const std::vector<MAN>& man_set, const std::vector<WOMAN> woman_set,
                                 std::function<MAN_RANK(const MAN& juge, const WOMAN& women)> man_rank,
                                 std::function<WOMAN_RANK(const WOMAN& juge, const MAN& man)> woman_rank,
                                 const bool man_rank_is_increasing = true, const bool woman_rank_is_increasing = true)
{
  Matchings result;

  std::map<unsigned int,                                   // MAN_ids
           std::vector<std::pair<unsigned int, MAN_RANK>>  // preferences list with rank of keys
           >
      table_of_man_preferences;
  for (unsigned int man_id = 0; man_id < man_set.size(); man_id++)
  {
    table_of_man_preferences.emplace(man_id, std::vector<std::pair<unsigned int, MAN_RANK>>(woman_set.size()));
    for (unsigned int women_id = 0; women_id < woman_set.size(); women_id++)
    {
      table_of_man_preferences.at(man_id)[women_id] =
          std::pair<unsigned int, MAN_RANK>(women_id, man_rank(man_set[man_id], woman_set[women_id]));
    }
    std::sort(table_of_man_preferences.at(man_id).begin(), table_of_man_preferences.at(man_id).end(),
              [&man_rank_is_increasing](const std::pair<unsigned int, MAN_RANK>& a,
                                        const std::pair<unsigned int, MAN_RANK>& b) {
                return (man_rank_is_increasing ? (a.second < b.second) : (a.second > b.second));
              });
  }

  typedef std::map<unsigned int,                        // woman id
                   std::pair<unsigned int, WOMAN_RANK>  // man id and it's corresponding woman notation
                   >
      Woman_to_man_matching;
  Woman_to_man_matching woman_to_man_matching;

  std::list<unsigned int> not_treated_mans;
  for (unsigned int man_id = 0; man_id < man_set.size(); man_id++)
  {
    not_treated_mans.push_back(man_id);
  }
  while (not_treated_mans.size() > 0)
  {
    unsigned int man_id = not_treated_mans.front();
    not_treated_mans.pop_front();
    bool affected = false;
    while ((!affected) and table_of_man_preferences.at(man_id).size() > 0)
    {
      std::pair<unsigned int, MAN_RANK> prefered_choice = table_of_man_preferences.at(man_id).back();
      unsigned int woman_id = prefered_choice.first;
      WOMAN_RANK man_value = woman_rank(woman_set[woman_id], man_set[man_id]);
      typename Woman_to_man_matching::const_iterator it = woman_to_man_matching.find(woman_id);
      if (it == woman_to_man_matching.end())
      {
        woman_to_man_matching.emplace(woman_id, std::pair<unsigned int, WOMAN_RANK>(man_id, man_value));
        affected = true;
      }
      else if (woman_rank_is_increasing ? it->second.second < man_value : it->second.second > man_value)
      {
        unsigned int previous_prefered_man_id = it->second.first;
        table_of_man_preferences.at(previous_prefered_man_id).pop_back();
        not_treated_mans.push_back(previous_prefered_man_id);
        woman_to_man_matching.at(woman_id) = std::pair<unsigned int, double>(man_id, man_value);
        affected = true;
      }
      else
      {
        table_of_man_preferences.at(man_id).pop_back();
      }
    }
  }

  unsigned int i = 0;
  for (const std::pair<int, std::pair<int, WOMAN_RANK>>& matching : woman_to_man_matching)
  {
    unsigned int woman_id = matching.first;
    unsigned int man_id = matching.second.first;
    result.women_to_man_matchings[woman_id] = man_id;
    result.man_to_women_matchings[man_id] = woman_id;
    i++;
  }
  for (unsigned int man_id = 0; man_id < man_set.size(); man_id++)
  {
    if (result.man_to_women_matchings.find(man_id) == result.man_to_women_matchings.end())
    {
      result.unaffected_man.push_back(man_id);
    }
  }
  for (unsigned int woman_id = 0; woman_id < woman_set.size(); woman_id++)
  {
    if (result.women_to_man_matchings.find(woman_id) == result.women_to_man_matchings.end())
    {
      result.unaffected_woman.push_back(woman_id);
    }
  }
  return result;
}

};  // namespace matching
};  // namespace RhobanSSL

#endif
