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

template <typename T, int N>
class CircularVector
{
private:
  T vector_[N];
  unsigned int index_;

public:
  CircularVector() : index_(0)
  {
  }
  CircularVector(unsigned int size) : index_(0)
  {
  }
  CircularVector(const CircularVector<T,N>& cv) : index_(cv.index_)
  {
      for(int i=0;i<N;++i)
       vector_[i]=cv.vector_[i];
  }

  unsigned int size() const
  {
    return N;
  }
  /*
  void resize(unsigned int size)
  {
    std::rotate(vector_, vector_ + index_, vector_+N);
    index_ = 0;
    vector_.resize(size);
  }
  */

  void insert(const T& element)
  {
    if (index_ == 0)
    {
      index_ = N;
    }
    index_ -= 1;
    vector_[index_] = element;
  }

  const T& operator[](unsigned int i) const
  {
    assert(i < N);
    if (index_ + i < N)
    {
      return vector_[index_ + i];
    }
    else
    {
      return vector_[index_ + i - N];
    }
  }
  T& operator[](unsigned int i)
  {
    assert(i < N);
    if (index_ + i < N)
    {
      return vector_[index_ + i];
    }
    else
    {
      return vector_[index_ + i - N];
    }
  }
  template <typename U,int P>
  friend std::ostream& operator<<(std::ostream& stream, const CircularVector<U,P>& vec);
};

template <typename T,int N>
std::ostream& operator<<(std::ostream& stream, const CircularVector<T,N>& vec)
{
  assert(N >= 1);
  stream << "(";
  for (unsigned int i = 0; i < N - 1; i++)
  {
    stream << vec[i] << ", ";
  }
  stream << vec[N - 1];
  stream << ")";
  return stream;
}
