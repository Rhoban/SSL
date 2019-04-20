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

#pragma once

#include <iostream>
#include <set>
#include <utility>
#include <list>
#include <vector>
#include <map>

namespace std
{
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::set<T>& set)
{
  out << "{";
  for (const std::string& elem : set)
  {
    out << elem << ", ";
  }
  out << "}";
  return out;
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::list<T>& list)
{
  out << "<";
  for (const T& elem : list)
  {
    out << elem << ", ";
  }
  out << ">";
  return out;
}

template <typename K, typename V>
std::ostream& operator<<(std::ostream& out, const std::map<K, V>& map)
{
  out << "{";
  for (const std::pair<K, V>& elem : map)
  {
    out << elem.first << " : " << elem.second << ", ";
  }
  out << "}";
  return out;
}

template <typename F, typename S>
std::ostream& operator<<(std::ostream& out, const std::pair<F, S>& pair)
{
  out << "(" << pair.first << ", " << pair.second << ")";
  return out;
}

template <typename V>
std::ostream& operator<<(std::ostream& out, const std::vector<V>& vector)
{
  out << "[";
  for (unsigned int i = 0; i < vector.size(); i++)
  {
    out << vector.at(i) << ", ";
  }
  out << "]";
  return out;
}

}  // namespace std
