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
};
}

#endif //def DYNAMIC_MATRIX_H
