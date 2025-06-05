/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

#ifndef DYNAMIC_MATRIX_H
#define DYNAMIC_MATRIX_H

#include "number.h"

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace FunctionalCalculator
{

template <typename Num>
class Matrix : public Number
{
  std::vector<std::vector<Num> > rows;

  static int getLeadingOneIndex(const std::vector<Num>& row)
  {
    Num zero_;
    int num_Cols = (int)row.size();
    int ii = 0;
    for (; ii < num_Cols; ++ii)
    {
      if (row[ii] != zero_) { return ii; }
    }
    return ii;
  }

  /** \brief Useful for resorting rows in matrix. */
  static bool compareLessThan(const std::vector<Num>& P, const std::vector<Num>& Q)
  {
    return getLeadingOneIndex(P) < getLeadingOneIndex(Q);
  }

public:

  virtual std::pair<double, double> get() const override
  {
    throw std::runtime_error("\nNot applicable.");
    return { 0.0, 0.0 };
  }

  virtual std::string print(bool useParentheses = false) const override
  {
    if (rows.empty()) { return (useParentheses ? "(0)" : "0"); }
    std::stringstream strm;
    int num_Rows = (int)rows.size();
    int num_Cols = (int)rows[0].size();
    std::vector<std::vector<std::string> > buffer(num_Rows);
    for (int ii = 0; ii < num_Rows; ++ii) { buffer[ii].resize(num_Cols); }
    std::vector<int> longestRows(num_Cols, 0);
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        std::stringstream strm0;
        strm0 << rows[ii][jj].print(false);
        auto current = strm0.str();
        buffer[ii][jj] = current;
        if ((int)(current.length()) > longestRows[jj]) { longestRows[jj] = (int)(current.length()); }
      }
    }
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      strm << "\n";
      if (useParentheses)
      {
        if (num_Rows == 1) { strm << "("; } else { strm << "|| "; }
      }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        if (jj > 0)
        {
          if (num_Rows == 1) { strm << ", "; } else { strm << " | "; }
        }
        const auto& current = buffer[ii][jj];
        int currentLength = (int)current.length();
        strm << current; // TODO: Deal with situation where entries themselves have multiple rows.
        for (int kk = 0; kk < (longestRows[jj] - currentLength); ++kk) { strm << " "; }
      }
      if (num_Rows == 1) { strm << ")"; } else { strm << " ||"; }
    }
    return strm.str();
  }

  int numRows() const { return (int)(rows.size()); }
  int numCols() const
  {
    if (rows.empty()) { return 0; }
    return (int)(rows[0].size());
  }

  Num at(int i, int j) const
  {
    if (i < 0) { throw std::invalid_argument("Row index cannot be negative."); return Num(); }
    if (j < 0) { throw std::invalid_argument("Columns index cannot be negative."); return Num(); }
    if (i >= numRows()) { throw std::invalid_argument("Row index must be less than matrix dimension."); return Num(); }
    if (j >= numCols()) { throw std::invalid_argument("Columns index must be less than matrix dimension."); return Num(); }
    return rows[i][j];
  }

  Matrix(const std::vector<Num>& rowIn = {})
  {
    rows.resize(0); if (!rowIn.empty()) { rows.push_back(rowIn); }
  }

  void addRow(const std::vector<Num>& rowIn)
  {
    if (rows.empty()) { rows.push_back(rowIn); return; }
    if (rowIn.size() != rows[0].size()) { throw std::invalid_argument("Rows must have equal length."); return; }
    rows.push_back(rowIn);
  }

  /** \brief Elementary row operation. Changes sign of the determinant if i != j. */
  void swapRows(int i, int j)
  {
    if (i < 0) { throw std::invalid_argument("Index out of bounds."); return; } // If size == 0 one of these always is called.
    if (j < 0) { throw std::invalid_argument("Index out of bounds."); return; }
    if (i >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    if (j >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    if (i == j) { return; }
    int nn = (int)(rows[0].size());
    for (int kk = 0; kk < nn; ++kk)
    {
      auto temp = rows[i][kk];
      rows[i][kk] = rows[j][kk];
      rows[j][kk] = temp;
    }
  }

  /** \brief Elementary row operation. Multiplies the determinant by scal. */
  void scaleRow(int i, const Num& scal)
  {
    if (i < 0) { throw std::invalid_argument("Index out of bounds."); return; } // If size == 0 one of these always is called.
    if (i >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    int nn = (int)(rows[0].size());
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = scal * rows[i][kk];
    }
  }

  /** \brief Elementary row operation. Leaves determinant unchanged if i != j, otherwise acts as scaleRow method by (1 + scal). */
  void addScaledRowJ_toI(int i, int j, const Num& scal)
  {
    if (i < 0) { throw std::invalid_argument("Index out of bounds."); return; } // If size == 0 one of these always is called.
    if (j < 0) { throw std::invalid_argument("Index out of bounds."); return; }
    if (i >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    if (j >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    int nn = (int)(rows[0].size());
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = rows[i][kk] + scal * rows[j][kk];
    }
  }

  bool isSquare() const
  {
    if (rows.empty()) { return true; }
    return (rows.size() == (rows[0].size()));
  }

  /** \return Reduced row echelon form.
   * 
   *  \param `determinant` Output reference is set to the determinant of the matrix, or 0 if matrix is not square.
   *  \param `linInd` Output reference is set to true if rows of the matrix are linearly independent
   *  \param `ignoreDeterminant` Speeds up algorithm by ignoring determinant (use O(n * log(n)) sort rather than bubble sort)
   */
  Matrix rref(Num& determinant, bool& linIndep, bool ignoreDeterminant = false) const
  {
    if (!ignoreDeterminant) { determinant = Num(); }
    linIndep = true;
    if (rows.empty()) { return *this; }
    Num unit(1);
    Num det = unit;
    int num_Rows = (int)rows.size();
    int num_Cols = (int)rows[0].size();
    Matrix answer;
    answer.rows = rows;
    bool gotToRowEchelon = false;
    for (bool performingRref = true; performingRref; performingRref = !performingRref)
    {
      if (ignoreDeterminant && (!gotToRowEchelon)) // Rearrange rows...
      {
        std::sort(answer.rows.begin(), answer.rows.end(), compareLessThan);
      }
      // else ... Bubble sort while computing determinant.
      for (int ii = 0; ii < num_Rows; ++ii)
      {
        if (ignoreDeterminant) { break; }
        if (gotToRowEchelon) { break; }
        int leadI = getLeadingOneIndex(answer.rows[ii]);
        for (int jj = ii + 1; jj < num_Rows; ++jj)
        {
          int leadJ = getLeadingOneIndex(answer.rows[jj]);
          if (leadI <= leadJ) { continue; }
          answer.swapRows(ii, jj);
          det = -det;
          performingRref = false;
          break;
        }
        if (!performingRref) { break; }
      }
      if (!performingRref) { continue; }
      bool adjusting = false;
      for (int ii = 0; ii < num_Rows; ++ii) // Divide by leading coefficient.
      {
        if (gotToRowEchelon) { break; }
        int jj = getLeadingOneIndex(answer.rows[ii]);
        if (jj >= num_Cols) { linIndep = false; break; }
        if (answer.rows[ii][jj] == unit) { continue; }
        auto factor = unit / answer.rows[ii][jj];
        answer.scaleRow(ii, factor);
        det = det * factor;
        adjusting = true;
      }
      // Subtract rows where applicable.
      {
        int prevInd = -1;
        if (!gotToRowEchelon) { prevInd = getLeadingOneIndex(answer.rows[0]); }
        for (int ii = 1; ii < num_Rows; ++ii)
        {
          if (gotToRowEchelon) { break; }
          int jj = getLeadingOneIndex(answer.rows[ii]);
          if (jj >= num_Cols) { linIndep = false; break; }
          if (jj > prevInd) { prevInd = jj; continue; }
          answer.addScaledRowJ_toI(ii, prevInd, -unit); // det unchanged.
          adjusting = true;
        }
      }
      if (adjusting) { performingRref = false; continue; }
      gotToRowEchelon = true;
      // Now from row echelon form, modify to reduced row echelon form.
      for (int ii = num_Rows - 1; ii >= 0; --ii)
      {
        int ind = getLeadingOneIndex(answer.rows[ii]);
        if (ind >= num_Cols) { linIndep = false; continue; }
        for (int jj = ii - 1; jj >= 0; --jj)
        {
          if (answer.rows[jj][ind] == Num()) { continue; }
          answer.addScaledRowJ_toI(jj, ii, -(answer.rows[jj][ind])); // det unchanged.
        }
      }
      // Now the answer is in rref.
    }
    if (isSquare()) { determinant = det; }
    return answer;
  }

  Num determinant() const
  {
    Num det; bool success = false;
    rref(det, success);
    return det;
  }

  /** \param `success` is true if and only matrix is invertible.
   *  \return Inverse matrix if the matrix is invertible, zero otherwise.
   */
  Matrix inverse(bool& success) const
  {
    Matrix answer;
    if (!isSquare()) { throw std::invalid_argument("Matrix must be square."); success = false; return answer; }
    const int dim = numRows();
    if (dim == 0) { success = false; return answer; }
    Num zero;
    Num unit(1);
    Matrix rREFed;
    for (int ii = 0; ii < dim; ++ii)
    {
      auto newRow = rows[ii];
      newRow.resize(2 * dim);
      for (int jj = 0; jj < dim; ++jj) { newRow[dim + jj] = ((ii == jj) ? unit : zero); }
      rREFed.addRow(newRow);
    }
    Num det;
    rREFed = rREFed.rref(det, success, true);
    if (!success) { return answer; }
    auto dimTwice = 2 * dim;
    for (int ii = 0; ii < dim; ++ii)
    {
      std::vector<Num> newRow(dim);
      auto& current = rREFed.rows[ii];
      for (int jj = 0; jj < dim; ++jj)
      {
        newRow[jj] = current[dim + jj];
      }
      answer.addRow(newRow);
    }
    return answer;
  }

  Matrix operator+() const { return *this; }
  Matrix operator-() const
  {
    Matrix answer;
    answer.rows = rows;
    for (auto& row : rows) { for (auto& item : row) { item = -item; } }
    return answer;
  }

  Matrix operator+(const Matrix& rhs) const
  {
    Matrix answer;
    if (rows.empty() && !(rhs.rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rhs.rows.empty() && !(rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows.size() != rhs.rows.size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows[0].size() != rhs.rows[0].size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    int num_Rows = (int)rows.size(); int num_Cols = (int)rows[0].size();
    answer.rows = rows;
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] + rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  Matrix operator-(const Matrix& rhs) const { return ((*this) + (-rhs)); }

  Matrix operator*(const Num& rhs) const
  {
    Matrix answer;
    answer.rows = rows;
    const int num_Rows = numRows();
    if (num_Rows == 0) { return answer; }
    const int num_Cols = numCols();
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] * rhs;
      }
    }
    return answer;
  }

  Matrix operator*(const Matrix& rhs) const
  {
    Matrix answer;

    int mm = (int)rows.size();
    if (mm == 0) { return answer; }
    int nn = (int)rows[0].size();
    if (nn != (int)rhs.rows.size()) { throw std::invalid_argument("Matrix mult A * B: numCols(A) must equal numRows(B)"); return answer; }
    int pp = (int)rhs.rows[0].size();

    answer.rows.resize(mm);
    for (int ii = 0; ii < mm; ++ii) { answer.rows[ii].resize(pp); }

    for (int ii = 0; ii < mm; ++ii)
    {
      for (int jj = 0; jj < pp; ++jj)
      {
        auto& current = answer.rows[ii][jj];
        for (int kk = 0; kk < nn; ++kk)
        {
          current = current + rows[ii][kk] * rhs.rows[kk][jj];
        }
      }
    }
    return answer;
  }

  Matrix transpose() const
  {
    Matrix answer;
    if (rows.empty()) { return answer; }

    int num_Rows = (int)rows.size();
    int num_Cols = (int)rows[0].size();

    answer.rows.resize(num_Cols);
    for (int ii = 0; ii < num_Cols; ++ii) { answer.rows[ii].resize(num_Rows); }

    for (int ii = 0; ii < num_Cols; ++ii) { for (int jj = 0; jj < num_Rows; ++jj) { answer.rows[ii][jj] = rows[jj][ii]; } }
    return answer;
  }

  bool operator==(const Matrix& rhs) const
  {
    if (rows.size() != rhs.rows.size()) { return false; }
    int num_Rows = (int)rows.size();
    int num_Cols = 0;
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      if (rows[ii].size() != rhs.rows[ii].size()) { return false; }
      if (ii > 0)
      {
        if (rows[0].size() != rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
        if (rhs.rows[0].size() != rhs.rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
      }
      else
      { num_Cols = (int)rows[0].size(); }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        if (rows[ii][jj] == rhs.rows[ii][jj]) { continue; }
        return false;
      }
    }
    return true;
  }

  bool operator!=(const Matrix& rhs) const
  {
    return !((*this) == rhs);
  }

  bool operator<(const Matrix& rhs) const
  {
    if (rows.size() < rhs.rows.size()) { return true; }
    if (rows.size() > rhs.rows.size()) { return false; }
    int num_Rows = (int)rows.size();
    int num_Cols = 0;
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      if (ii > 0)
      {
        if (rows[0].size() != rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
        if (rhs.rows[0].size() != rhs.rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
      }
      else
      {
        if (rows[0].size() < rhs.rows[0].size()) { return true; }
        if (rows[0].size() > rhs.rows[0].size()) { return false; }
        num_Cols = (int)rows[0].size();
      }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        auto& aa = rows[ii][jj];
        auto& bb = rhs.rows[ii][jj];
        if (aa < bb) { return true; }
        if (aa == bb) { continue; }
        return false;
      }
    }
    return false;
  }

  bool operator>(const Matrix& rhs) const
  {
    return (rhs < (*this));
  }

  bool operator<=(const Matrix& rhs) const
  {
    if ((*this) < rhs) { return true; }
    if ((*this) == rhs) { return true; }
    return false;
  }

  bool operator>=(const Matrix& rhs) const
  {
    if (rhs < (*this)) { return true; }
    if (rhs == (*this)) { return true; }
    return false;
  }

  Num matrixDot(const Matrix& rhs) const
  {
    Num answer;
    answer = answer - answer;
    if (rows.empty() && !(rhs.rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rhs.rows.empty() && !(rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows.size() != rhs.rows.size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows[0].size() != rhs.rows[0].size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    int num_Rows = (int)rows.size(); int num_Cols = (int)rows[0].size();
    if ((num_Rows == 0) || (num_Cols == 0)) { return answer; }
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer = answer + rows[ii][jj] * rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  Num matrixSqNorm() const { return matrixDot(*this); }
};
}

#endif //def DYNAMIC_MATRIX_H
