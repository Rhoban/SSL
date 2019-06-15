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

#include <vector>
#include <algorithm>
#include <assert.h>
#include <ostream>

template <typename T>
class CircularVector
{
private:
  std::vector<T> vector_;
  unsigned int index_;
  unsigned int nb_elements_;

public:
  CircularVector() : vector_(0), index_(0), nb_elements_(0)
  {
  }
  CircularVector(unsigned int size) : vector_(size), index_(0), nb_elements_(0)
  {
  }
  CircularVector(const CircularVector<T>& cv) : vector_(cv.vector_), index_(cv.index_), nb_elements_(0)
  {
  }

  /**
   * @brief returns the number of element that have been inserted in the circular vector.
   * @return the number of elements in the circular vector.
   */
  unsigned int numberOfElements() const
  {
    return nb_elements_;
  }

  void clear()
  {
    nb_elements_ = 0;
    unsigned int max_size = size();
    vector_.clear();
    vector_.resize(max_size);
  }

  unsigned int size() const
  {
    return vector_.size();
  }

  void resize(unsigned int size)
  {
    std::rotate(vector_.begin(), vector_.begin() + index_, vector_.end());
    index_ = 0;
    if (nb_elements_ >= size)
      nb_elements_ = size;
    vector_.resize(size);
  }

  void insert(const T& element)
  {
    if (index_ == 0)
    {
      index_ = vector_.size();
    }
    index_ -= 1;
    if (nb_elements_ < size())
      nb_elements_++;
    vector_[index_] = element;
  }

  const T& operator[](unsigned int i) const
  {
    assert(i < vector_.size());
    if (index_ + i < vector_.size())
    {
      return vector_[index_ + i];
    }
    else
    {
      return vector_[index_ + i - vector_.size()];
    }
  }
  T& operator[](unsigned int i)
  {
    assert(i < vector_.size());
    if (index_ + i < vector_.size())
    {
      return vector_[index_ + i];
    }
    else
    {
      return vector_[index_ + i - vector_.size()];
    }
  }
  template <typename U>
  friend std::ostream& operator<<(std::ostream& stream, const CircularVector<U>& vec);
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const CircularVector<T>& vec)
{
  assert(vec.size() >= 1);
  stream << "(";
  for (unsigned int i = 0; i < vec.size() - 1; i++)
  {
    stream << vec[i] << ", ";
  }
  stream << vec[vec.size() - 1];
  stream << ")";
  return stream;
}
