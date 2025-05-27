/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "fn_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
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

  FnPolynomial::FnPolynomial(const ComplexQuadratic& coeff, int power)
  {
    if (power < 0) { throw std::invalid_argument("Exponent must be nonnegative."); }
    else if (coeff != 0)
    {
      self[power] = coeff;
    }
  }

  FnPolynomial::FnPolynomial(const std::vector<ComplexQuadratic>& coeffs)
  {
    for (int ii = 0; ii < (int)(coeffs.size()); ++ii) { if (coeffs[ii] != 0) { self[ii] = coeffs[ii]; } }
  }

  std::pair<double, double> FnPolynomial::get() const
  {
    double answerRe = 0.0;
    double answerIm = 0.0;
    for (const auto& iter : self)
    {
      auto monomial = std::pow(piValue(), iter.first);
      answerRe += iter.second.getRe().get().first * monomial;
      answerIm += iter.second.getIm().get().first * monomial;
    }
    return { answerRe, answerIm };
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
      if (iter.first == 1) { strm << "Pi"; }
      else if (iter.first > 1) { strm << "(Pi)"; }
      if (iter.first > 1) { strm << "^" << iter.first; }
    }
    if (count < 0) { strm << "0"; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  double FnPolynomial::piValue() { return 3.14159265359; }

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
    answer.self[0] = 1;
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
