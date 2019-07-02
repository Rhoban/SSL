/*
    This file is part of SSL.

    Copyright 2019 Georges Nicolas (ngeorges@enseirb-matmeca.fr)

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

/*
This file contains various tests for the matrix manipulation function implemented. Tests return 1 when success and 0
when failure
*/
#include "gsl_matrix.h"
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

using namespace rhoban_ssl;

bool eq(double u, double v, double err)
{
  return std::fabs(u - v) <= err;
}

int testInitialisation()
{
  GslMatrix m = GslMatrix(7, 3);
  double error = 0.0000001;

  // Dimensions check
  size_t rows = m.getRows();
  if (rows != 7)
    return 0;
  size_t cols = m.getColumns();
  if (cols != 3)
    return 0;

  // Values check
  for (int i = 0; i < 7; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      double elem = m.getElement(i, j);
      if (!(eq(elem, 0.0, error)))
        return 0;
    }
  }
  return 1;
}

int testSetAndGet()
{
  GslMatrix m = GslMatrix(6, 4);
  double error = 0.0000001;

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      m.setElement(i, j, i * 6.0 + j);
      double elem = m.getElement(i, j);
      if (!(eq(elem, i * 6.0 + j, error)))
        return 0;
    }
  }
  return 1;
}

int testGlobalSetters()
{
  GslMatrix m = GslMatrix(6, 4);
  double error = 0.0000001;

  m.setAll(1.2);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (!(eq(m.getElement(i, j), 1.2, error)))
        return 0;
    }
  }

  m.setIdentity();
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (j == i)
      {
        if (!(eq(m.getElement(i, j), 1.0, error)))
          return 0;
      }
      else
      {
        if (!(eq(m.getElement(i, j), 0.0, error)))
          return 0;
      }
    }
  }

  m.setZero();
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (!(eq(m.getElement(i, j), 0.0, error)))
        return 0;
    }
  }

  return 1;
}

int testBasicAssessors()
{
  GslMatrix m = GslMatrix(6, 4);
  if (!(m.isNull()))
    return 0;

  m.setAll(1.0);
  if (!(m.isPos()))
    return 0;

  m.setAll(-1.0);
  if (!(m.isNeg()))
    return 0;

  return 1;
}

int testCopy()
{
  GslMatrix m = GslMatrix(4, 3);
  GslMatrix n = GslMatrix(4, 3);
  double error = 0.0000001;

  m.setAll(2.0);
  m.copyMatrix(&n);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (!(eq(n.getElement(i, j), 2.0, error)))
        return 0;
    }
  }
  if (&m == &n)
    return 0;

  return 1;
}

int testTranspose()
{
  GslMatrix m = GslMatrix(5, 4);
  GslMatrix n = GslMatrix(4, 5);
  double error = 0.0000001;

  m.setElement(4, 3, 2.0);
  m.setElement(1, 1, 1.0);
  m.transposeMatrix(&n);

  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 5; i++)
    {
      if ((i == 4) && (j == 3))
      {
        if (!(eq(n.getElement(i, j), 2.0, error)))
          return 0;
      }
      if ((i == 1) && (j == 1))
      {
        if (!(eq(n.getElement(i, j), 1.0, error)))
          return 0;
      }
      else
      {
        if (!(eq(n.getElement(i, j), 1.0, error)))
          return 0;
      }
    }
  }
  return 1;
}

int testOperations()
{
  GslMatrix m = GslMatrix(6, 3);
  GslMatrix n = GslMatrix(3, 6);
  GslMatrix n2 = GslMatrix(3, 6);
  GslMatrix n3 = GslMatrix(3, 6);
  GslMatrix o = GslMatrix(3, 3);
  double error = 0.0000001;

  // test addition
  n.setAll(1.0);
  n2.setAll(1.0);

  n2.addMatrix(&n);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      if (!(eq(n.getElement(i, j), 2.0, error)))
        return 0;
    }
  }

  // test substraction
  n3.setAll(2.0);
  n3.subMatrix(&n);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      if (!(eq(n.getElement(i, j), 0.0, error)))
        return 0;
    }
  }

  // test scale matrix
  // test BLAS matrix product
  // test BLASKalman matrix product
  return 1;
}

int testPositiveDefined()
{
  GslMatrix m = GslMatrix(2, 2);
  m.setIdentity();
  if (!(m.isPositiveDefinite()))
  {
    return 0;
  }

  m.setElement(0, 0, 1);
  m.setElement(0, 1, 2);
  m.setElement(1, 0, -2);
  m.setElement(1, 1, 4);
  if (!(m.isPositiveDefinite()))
  {
    return 0;
  }

  return 1;
}

int testInversible()
{
  GslMatrix m = GslMatrix(2, 2);
  GslMatrix n = GslMatrix(2, 2);
  double error = 0.0000001;

  m.setElement(0, 0, 1);
  m.setElement(0, 1, 2);
  m.setElement(1, 0, -2);
  m.setElement(1, 1, 4);
  m.inverseMatrix(&n);

  double n1 = n.getElement(0, 0);
  double n2 = n.getElement(0, 1);
  double n3 = n.getElement(1, 0);
  double n4 = n.getElement(1, 1);

  if (!(eq(n1, 0.5, error)))
  {
    return 0;
  }
  if (!(eq(n2, -0.25, error)))
  {
    return 0;
  }
  if (!(eq(n3, 0.25, error)))
  {
    return 0;
  }
  if (!(eq(n4, 0.125, error)))
  {
    return 0;
  }

  return 1;
}
 


int main()
{
  printf("Now testing gsl_matrix...");
  assert(testInitialisation());
  assert(testSetAndGet());
  assert(testGlobalSetters());
  assert(testBasicAssessors());
  assert(testCopy());
  assert(testTranspose());
  assert(testOperations());
  assert(testPositiveDefined());
  assert(testInversible());
  printf("successful\n");
  return EXIT_SUCCESS;
}