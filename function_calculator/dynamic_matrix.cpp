/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

#include "dynamic_matrix.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  std::pair<double, double> Matrix::get() const
  {
    throw std::runtime_error("\nNot applicable.");
    return { 0.0, 0.0 };
  }

  std::string Matrix::print(bool useParentheses) const
  {
    if (rows.empty()) { return (useParentheses ? "(0)" : "0"); }
    std::stringstream strm;
    int numRows = (int)rows.size();
    for (const auto& row : rows)
    {
      if (useParentheses)
      {
        if (numRows == 1) { strm << "("; } else { strm << "\n| "; }
      }
      int ii = -1;
      for (const auto& item : row)
      {
        ++ii;
        if (ii > 0)
        {
          if (numRows == 1) { strm << ", "; } else { strm << " "; }
        }
        strm << item.print(false); // TODO: Deal with situation where entries themselves have multiple rows.
      }
      if (numRows == 1) { strm << ")"; } else { strm << " |"; }
    }
    return strm.str();
  }
}
