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

#include "gsl_matrix.h"

namespace rhoban_ssl
{
GslMatrix::GslMatrix(const size_t i, const size_t j)
{
  m_ = gsl_matrix_calloc(i, j);
}

// global setters
void GslMatrix::setAll(double x)
{
  gsl_matrix_set_all(m_, x);
}

void GslMatrix::setZero()
{
  gsl_matrix_set_zero(m_);
}

void GslMatrix::setIdentity()
{
  gsl_matrix_set_identity(m_);
}

// row and col getters

size_t GslMatrix::getRows() const
{
  return this->m_->size1;
}

size_t GslMatrix::getColumns() const
{
  return this->m_->size2;
}

// element get and set
double GslMatrix::getElement(const size_t i, const size_t j) const
{
  return gsl_matrix_get(m_, i, j);
}

void GslMatrix::setElement(const size_t i, const size_t j, double x)
{
  gsl_matrix_set(m_, i, j, x);
}

// copy matrix
int GslMatrix::copyMatrix(GslMatrix* dest) const
{
  return gsl_matrix_memcpy(dest->m_, this->m_);
}

// transpose matrix
int GslMatrix::transposeMatrix(GslMatrix* dest) const
{
  return gsl_matrix_transpose_memcpy(dest->m_, this->m_);
}

// basic linear operations in matrices
int GslMatrix::addMatrix(GslMatrix* a) const
{
  return gsl_matrix_add(a->m_, this->m_);
}
int GslMatrix::subMatrix(GslMatrix* a) const
{
  return gsl_matrix_sub(a->m_, this->m_);
}

int GslMatrix::scaleMatrix(const double x)
{
  return gsl_matrix_scale(this->m_, x);
}

int GslMatrix::multMatrixBLAS(GslMatrix* dest, const GslMatrix* a, const GslMatrix* c, double alpha, double beta,
                              bool transposeA, bool transposeB) const
{
  // The receiving matrix is matrix C in BLAS implementation, so dest is set up as a copy of C
  c->copyMatrix(dest);

  // Setup of parameters for transposition of matrix or not in the BLAS execution
  CBLAS_TRANSPOSE_t TransA;
  CBLAS_TRANSPOSE_t TransB;
  if (transposeA == false)
  {
    TransA = CblasNoTrans;
  }
  else
  {
    TransA = CblasTrans;
  }
  if (transposeB == false)
  {
    TransB = CblasNoTrans;
  }
  else
  {
    TransB = CblasTrans;
  }
  // execution of matrix-matrix product and sum
  return (gsl_blas_dgemm(TransA, TransB, alpha, a->m_, m_, beta, dest->m_));
}

int GslMatrix::multMatrixBLASKalman(GslMatrix* dest, const GslMatrix* a, const GslMatrix* c, const GslMatrix* d,
                                    double alpha, double beta, bool transA, bool transB, bool transC) const
{
  int rc = 1;
  int rc2 = 1;
  size_t inter_rows_number = 0;
  size_t inter_cols_number = 0;
  if (transB)
  {
    if (transA)
    {
    }
    else
    {
      inter_rows_number = this->getColumns();
      inter_cols_number = c->getColumns();
    }
  }
  else
  {
    if (transA)
    {
      inter_rows_number = this->getRows();
      inter_cols_number = c->getRows();
    }
    else
    {
      inter_rows_number = this->getRows();
      inter_cols_number = c->getColumns();
    }
  }
  GslMatrix intermediar_matrix = GslMatrix(inter_rows_number, inter_cols_number);
  GslMatrix null_matrix = GslMatrix(inter_rows_number, inter_cols_number);
  rc = c->multMatrixBLAS(&intermediar_matrix, this, &null_matrix, 1.0, 0.0, transB, transC);
  rc2 = intermediar_matrix.multMatrixBLAS(dest, a, d, alpha, beta, transA, false);

  return (rc || rc2);
}

// standards evaluators (POSIX norm : 0 -> success)
bool GslMatrix::isNull() const
{
  int value = gsl_matrix_isnull(m_);
  if (value == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool GslMatrix::isPos() const
{
  int value = gsl_matrix_ispos(m_);
  if (value == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool GslMatrix::isNeg() const
{
  int value = gsl_matrix_isneg(m_);
  if (value == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Cholesky verification of defined positivity of a matrix (POSIX norm : 0 -> success)

bool GslMatrix::isPositiveDefinite() const
{
  // deactivate the error handler
  gsl_error_handler_t* previous_handler = gsl_set_error_handler_off();

  // copy the matrix
  size_t rows = this->getRows();
  size_t cols = this->getColumns();
  GslMatrix copy = GslMatrix(rows, cols);
  this->copyMatrix(&copy);

  // use Cholesky to check if it is defined positive ; the Cholesky decomposition will be discarded afterward
  int rc = gsl_linalg_cholesky_decomp1(copy.m_);

  // reactivate the error handler
  gsl_set_error_handler(previous_handler);

  // return the boolean
  if (rc == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int GslMatrix::inverseMatrix(GslMatrix* dest) const
{
  // deactivate the error handler
  gsl_error_handler_t* previous_handler = gsl_set_error_handler_off();

  this->copyMatrix(dest);
  // cholesky decomp

  int rc = gsl_linalg_cholesky_decomp1(dest->m_);

  // cholesky invert
  int rc2 = gsl_linalg_cholesky_invert(dest->m_);

  // reactivate the error handler
  gsl_set_error_handler(previous_handler);

  return (rc || rc2);  // 0 when success, non-zero value when failure + gsl error code
}

GslMatrix::~GslMatrix()
{
  gsl_matrix_free(m_);
}

}  // namespace rhoban_ssl