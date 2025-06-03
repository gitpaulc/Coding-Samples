/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

#ifndef DYNAMIC_MATRIX_H
#define DYNAMIC_MATRIX_H

#include "number.h"

#include <stdexcept>
#include <sstream>
#include <vector>

namespace FunctionalCalculator
{

template <typename Num>
class Matrix : public Number
{
  std::vector<std::vector<Num> > rows;
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
    int numRows = (int)rows.size();
    int numCols = (int)rows[0].size();
    std::vector<std::vector<std::string> > buffer(numRows);
    for (int ii = 0; ii < numRows; ++ii) { buffer[ii].resize(numCols); }
    std::vector<int> longestRows(numCols, 0);
    for (int ii = 0; ii < numRows; ++ii)
    {
      for (int jj = 0; jj < numCols; ++jj)
      {
        std::stringstream strm0;
        strm0 << rows[ii][jj].print(false);
        auto current = strm0.str();
        buffer[ii][jj] = current;
        if ((int)(current.length()) > longestRows[jj]) { longestRows[jj] = (int)(current.length()); }
      }
    }
    for (int ii = 0; ii < numRows; ++ii)
    {
      if (useParentheses)
      {
        if (numRows == 1) { strm << "("; } else { strm << "\n|| "; }
      }
      for (int jj = 0; jj < numCols; ++jj)
      {
        if (jj > 0)
        {
          if (numRows == 1) { strm << ", "; } else { strm << " | "; }
        }
        const auto& current = buffer[ii][jj];
        int currentLength = (int)current.length();
        strm << current; // TODO: Deal with situation where entries themselves have multiple rows.
        for (int kk = 0; kk < (longestRows[jj] - currentLength); ++kk) { strm << " "; }
      }
      if (numRows == 1) { strm << ")"; } else { strm << " ||"; }
    }
    return strm.str();
  }

  void addRow(const std::vector<Num>& rowIn)
  {
    if (rows.empty()) { rows.push_back(rowIn); return; }
    if (rowIn.size() != rows[0].size()) { throw std::invalid_argument("Rows must have equal length."); return; }
    rows.push_back(rowIn);
  }

  Matrix operator+() const { return *this; }
  Matrix operator-() const
  {
    Matrix answer;
    answer.rows = rows;
    for (auto& row : rows) { for (auto& item : row) { item = -item; } }
    return answer;
  }

  Matrix operator+(const Matrix& rhs)
  {
    Matrix answer;
    if (rows.empty() && !(rhs.rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rhs.rows.empty() && !(rows.empty())) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows.size() != rhs.rows.size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    if (rows[0].size() != rhs.rows[0].size()) { throw std::invalid_argument("Matrix sizes must be equal."); return answer; }
    int numRows = (int)rows.size(); int numCols = (int)rows[0].size();
    answer.rows = rows;
    for (int ii = 0; ii < numRows; ++ii)
    {
      for (int jj = 0; jj < numCols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] + rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  Matrix operator-(const Matrix& rhs) const { return ((*this) + (-rhs)); }

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

    int numRows = (int)rows.size();
    int numCols = (int)rows[0].size();

    answer.rows.resize(numCols);
    for (int ii = 0; ii < numCols; ++ii) { answer.rows[ii].resize(numRows); }

    for (int ii = 0; ii < numCols; ++ii) { for (int jj = 0; jj < numRows; ++jj) { answer.rows[ii][jj] = rows[jj][ii]; } }
    return answer;
  }
};
}

#endif //def DYNAMIC_MATRIX_H
