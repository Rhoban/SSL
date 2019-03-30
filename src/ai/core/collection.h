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

#ifndef __CORE__COLLECTION__H__
#define __CORE__COLLECTION__H__

#include <list>
#include <vector>
#include <map>
#include <set>
#include <functional>

template <typename DATA>
std::vector<DATA> list2vector(const std::list<DATA>& list, size_t vector_size)
{
  assert(vector_size >= list.size());
  std::vector<DATA> vec(vector_size);
  unsigned int i = 0;
  for (const DATA& data : list)
  {
    vec[i] = data;
    i++;
  }
  return vec;
}

template <typename DATA>
std::vector<DATA> list2vector(const std::list<DATA>& list)
{
  return list2vector(list, list.size());
}

template <typename DATA>
std::set<DATA> vector2set(const std::vector<DATA>& vec)
{
  std::set<DATA> res;
  for (const DATA& data : vec)
  {
    res.insert(data);
  }
  return res;
}

template <typename KEY, typename VALUE>
std::vector<std::pair<KEY, VALUE> > map2vector(const std::map<KEY, VALUE>& map)
{
  std::vector<std::pair<KEY, VALUE> > result;
  int i = 0;
  for (const std::pair<KEY, VALUE>& assoc : map)
  {
    result[i] = assoc;
    i++;
  }
  return result;
}

template <typename KEY, typename VALUE>
std::set<std::pair<KEY, VALUE> > map2set(const std::map<KEY, VALUE>& map)
{
  std::set<std::pair<KEY, VALUE> > result;
  for (const std::pair<KEY, VALUE>& assoc : map)
  {
    result.insert(assoc);
  }
  return result;
}

template <typename DATA_SRC, typename DATA_DEST>
std::vector<DATA_DEST> map2vector(const std::vector<DATA_SRC>& source,
                                  std::function<void(const DATA_SRC&, DATA_DEST&)> transformation)
{
  std::vector<DATA_DEST> result(source.size());
  for (unsigned int i = 0; i < source.size(); i++)
  {
    transformation(source[i], result[i]);
  }
  return result;
}

template <typename DATA_SRC, typename DATA_DEST>
std::list<DATA_DEST> map2list(const std::vector<DATA_SRC>& source,
                              std::function<DATA_DEST(const DATA_SRC&)> transformation)
{
  std::list<DATA_DEST> result;
  for (unsigned int i = 0; i < source.size(); i++)
  {
    result.push_back(transformation(source[i]));
  }
  return result;
}

#endif
