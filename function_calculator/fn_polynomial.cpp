/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "fn_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  bool FnPolynomial::Monomial::isConstTerm() const
  {
    if (xInd != 0) { return false; }
    if (yInd != 0) { return false; }
    if (zInd != 0) { return false; }
    if (ePiXInd != 0) { return false; }
    if (ePiYInd != 0) { return false; }
    if (ePiZInd != 0) { return false; }
    return true;
  }

  FnPolynomial::Monomial FnPolynomial::Monomial::operator+(const FnPolynomial::Monomial& rhs) const
  {
    Monomial answer;
    answer.xInd = xInd + rhs.xInd;
    answer.yInd = yInd + rhs.yInd;
    answer.zInd = zInd + rhs.zInd;
    answer.ePiXInd = ePiXInd + rhs.ePiXInd;
    answer.ePiYInd = ePiYInd + rhs.ePiYInd;
    answer.ePiZInd = ePiZInd + rhs.ePiZInd;
    return answer;
  }

  bool FnPolynomial::Monomial::operator<(const FnPolynomial::Monomial& rhs) const
  {
    if (xInd < rhs.xInd) { return true; }
    if (xInd > rhs.xInd) { return false; }
    if (yInd < rhs.yInd) { return true; }
    if (yInd > rhs.yInd) { return false; }
    if (zInd < rhs.zInd) { return true; }
    if (zInd > rhs.zInd) { return false; }
    if (ePiXInd < rhs.ePiXInd) { return true; }
    if (ePiXInd > rhs.ePiXInd) { return false; }
    if (ePiYInd < rhs.ePiYInd) { return true; }
    if (ePiYInd > rhs.ePiYInd) { return false; }
    if (ePiZInd < rhs.ePiZInd) { return true; }
    if (ePiZInd > rhs.ePiZInd) { return false; }
    return false; // They are equal.
  }

  void FnPolynomial::clean()
  {
    FnPolynomial answer;
    for (const auto& iter : self)
    {
      if (iter.second == 0) { continue; }
      answer.self[iter.first] = iter.second;
    }
    self = answer.self;
  }

  FnPolynomial::FnPolynomial(const ComplexQuadratic& coeff)
  {
    if (coeff != 0)
    {
      Monomial constTerm;
      self[constTerm] = coeff;
    }
  }

  std::string FnPolynomial::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    int count = -1;
    for (const auto& iter : self)
    {
      if (iter.second == ComplexQuadratic()) { continue; }
      ++count;
      if (count != 0) { strm << " + "; }
      strm << iter.second.print(true);
      if (iter.first.isConstTerm()) { continue; }
      strm << " * ";
      if (iter.first.xInd != 0) { strm << "x^" << iter.first.xInd; }
      if (iter.first.yInd != 0) { strm << "y^" << iter.first.yInd; }
      if (iter.first.zInd != 0) { strm << "z^" << iter.first.zInd; }
      if (iter.first.ePiXInd != 0) { strm << "e^{Pi * " << iter.first.ePiXInd.print(true) << " * x}"; }
      if (iter.first.ePiYInd != 0) { strm << "e^{Pi * " << iter.first.ePiYInd.print(true) << " * y}"; }
      if (iter.first.ePiZInd != 0) { strm << "e^{Pi * " << iter.first.ePiZInd.print(true) << " * z}"; }
    }
    if (count < 0) { strm << "0"; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  FnPolynomial FnPolynomial::operator+() const
  {
    return *this;
  }

  FnPolynomial FnPolynomial::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.self)
    {
      iter.second = -iter.second;
    }
    return answer;
  }

  FnPolynomial FnPolynomial::operator+(const FnPolynomial& rhs) const
  {
    FnPolynomial answer = *this;

    for (auto& iter : rhs.self)
    {
      if (answer.self.find(iter.first) == answer.self.end())
      {
        answer.self[iter.first] = iter.second;
        continue;
      }
      answer.self[iter.first] = answer.self[iter.first] + iter.second;
    }
    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::operator-(const FnPolynomial& rhs) const
  {
    return (*this) + (-rhs);
  }

  FnPolynomial FnPolynomial::operator*(const FnPolynomial& rhs) const
  {
    FnPolynomial answer;

    for (const auto& iter : self)
    {
      for (const auto& jter : rhs.self)
      {
        auto summand = iter.second * jter.second;
        auto kk = iter.first + jter.first;
        if (answer.self.find(kk) == answer.self.end())
        {
          answer.self[kk] = summand;
          continue;
        }
        answer.self[kk] = answer.self[kk] + summand;
      }
    }
    return answer;
  }

  FnPolynomial FnPolynomial::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    FnPolynomial answer;
    Monomial constTerm;
    answer.self[constTerm] = 1;
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool FnPolynomial::operator==(const FnPolynomial& rhs) const
  {
    auto diff = (*this) - rhs;
    for (const auto& iter : diff.self)
    {
      if (iter.second != 0) { return false; }
    }
    return true;
  }

  bool FnPolynomial::operator!=(const FnPolynomial& rhs) const
  {
    return !((*this) == rhs);
  }
}
