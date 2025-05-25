/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "quadratic_number.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  double QuadraticNumber::get() const
  {
    double answer = 0.0;
    for (const auto& iter : content)
    {
      double val = iter.second.get();
      if (val == 0) { continue; }
      double radicand = iter.first;
      if (iter.first < 0) { throw std::exception("\nStill need to implement complex numbers."); radicand = -radicand; }
      answer += val * std::sqrt(radicand);
    }
    return answer;
  }

  std::string QuadraticNumber::print() const
  {
    std::stringstream strm;
    int count = -1;
    strm << "(";
    for (const auto& iter : content)
    {
      auto val = iter.second;
      if (val == 0) { continue; }
      ++count;
      if (count > 0)
      {
        if (val >= 0) { strm << " + "; }
        else
        {
          val = -val;
          strm << " - ";
        }
      }
      strm << val.print();
      if (val == 1) { continue; }
      int radicand = iter.first;
      bool complex = false;
      if (radicand < 0) { radicand = -radicand; complex = true; }
      strm << " * Sqrt(" << radicand;
      strm << ")";
      if (complex) { strm << " * i"; }
    }
    strm << ")";
    return strm.str();
  }
}
