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
    for (const auto& row : rows)
    {
      if (useParentheses)
      {
        if (numRows == 1) { strm << "("; } else { strm << "\n|| "; }
      }
      int ii = -1;
      for (const auto& item : row)
      {
        ++ii;
        if (ii > 0)
        {
          if (numRows == 1) { strm << ", "; } else { strm << " | "; }
        }
        strm << item.print(false); // TODO: Deal with situation where entries themselves have multiple rows.
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
