/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "pi_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  void PiPolynomial::clean()
  {
    PiPolynomial answer;
    for (const auto& iter : self)
    {
      if (iter.second == 0) { continue; }
      answer.self[iter.first] = iter.second;
    }
    self = answer.self;
  }

  PiPolynomial::PiPolynomial(const ComplexQuadratic& coeff, int power)
  {
    if (power < 0) { throw std::invalid_argument("Exponent must be nonnegative."); }
    else if (coeff != 0)
    {
      self[power] = coeff;
    }
  }

  PiPolynomial::PiPolynomial(const std::vector<ComplexQuadratic>& coeffs)
  {
    for (int ii = 0; ii < (int)(coeffs.size()); ++ii) { if (coeffs[ii] != 0) { self[ii] = coeffs[ii]; } }
  }

  std::pair<double, double> PiPolynomial::get() const
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

  std::string PiPolynomial::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    int count = -1;
    for (const auto& iter : self)
    {
      if (iter.second == ComplexQuadratic()) { continue; }
      ++count;
      bool contentIsOne = (iter.second == 1);
      bool contentIsMinusOne = (iter.second == -1);
      auto content = iter.second;
      std::string sumSign = "";
      if (count != 0) { sumSign = " + "; }
      if (contentIsMinusOne)
      {
        sumSign = ((count == 0) ? "-" : " - ");
        content = -content;
      }
      strm << sumSign;
      bool useBrackets = (!contentIsOne && !contentIsMinusOne);
      if (!contentIsOne && !contentIsMinusOne)
      {
        strm << content.print(useBrackets);
        if (iter.first > 0) { strm << " * "; }
      }
      else if (iter.first == 0) { strm << content.print(useBrackets); }
      if (iter.first == 1) { strm << "Pi"; }
      else if (iter.first > 1) { strm << "(Pi)"; }
      if (iter.first > 1) { strm << "^" << iter.first; }
    }
    if (count < 0) { strm << "0"; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  double PiPolynomial::piValue() { return 3.14159265359; }

  PiPolynomial PiPolynomial::operator+() const
  {
    return *this;
  }

  PiPolynomial PiPolynomial::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.self)
    {
      iter.second = -iter.second;
    }
    return answer;
  }

  PiPolynomial PiPolynomial::operator+(const PiPolynomial& rhs) const
  {
    PiPolynomial answer = *this;

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

  PiPolynomial PiPolynomial::operator-(const PiPolynomial& rhs) const
  {
    return (*this) + (-rhs);
  }

  PiPolynomial PiPolynomial::operator*(const PiPolynomial& rhs) const
  {
    PiPolynomial answer;

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

  PiPolynomial PiPolynomial::operator*(const Rational& rhs) const
  {
    PiPolynomial answer;
    for (const auto& iter : self) { answer.self[iter.first] = iter.second * ComplexQuadratic(rhs); }
    return answer;
  }

  int PiPolynomial::degree() const
  {
    int maxKey = 0;
    for (const auto& iter : self) { if (iter.first > maxKey) { maxKey = iter.first; } }
    return maxKey;
  }

  PiPolynomial PiPolynomial::division(const PiPolynomial& rhs, PiPolynomial& remainder) const
  {
    if (rhs == PiPolynomial(0))
    {
      if ((*this) == PiPolynomial(0)) { remainder = PiPolynomial(0);  return PiPolynomial(1); }
      throw std::invalid_argument("Division by zero.");
      remainder = PiPolynomial(0);  return PiPolynomial(1);
    }
    auto rhsDegree = rhs.degree();
    if (rhsDegree == 0)
    {
      auto quotient = *this;
      ComplexQuadratic coeff = ComplexQuadratic(1) / rhs.self.at(0);
      for (auto& iter : quotient.self) { iter.second = iter.second * coeff; }
      remainder = PiPolynomial(0);
      return quotient;
    }
    auto dividend = *this;
    auto divDegree = dividend.degree();
    PiPolynomial quotient = PiPolynomial(0);
    for (int prevDegree = divDegree; rhsDegree <= divDegree;)
    {
      auto monomial = PiPolynomial(rhs.self.at(rhsDegree) / dividend.self.at(divDegree), divDegree - rhsDegree);
      quotient = quotient + monomial;
      auto product = rhs * monomial;
      if (product == dividend) { remainder = PiPolynomial(0); return quotient; }
      dividend = dividend - product;
      prevDegree = divDegree;
      divDegree = dividend.degree();
      if (prevDegree <= divDegree) { break; } // Should never happen.
    }
    remainder = dividend;
    return quotient;
  }

  PiPolynomial PiPolynomial::gcd(const PiPolynomial& aa, const PiPolynomial& bb)
  {
    if ((aa == bb) || (aa == PiPolynomial(0))) { return bb; }
    if (bb == PiPolynomial(0)) { return aa; }
    auto aPoly = aa;
    auto bPoly = bb;
    while (bPoly != PiPolynomial(0))
    {
      auto aa_old = aPoly;
      auto bb_old = bPoly;
      aPoly = bb_old;
      auto quotient = aa_old.division(bb_old, bPoly);
    }
    if (aPoly == PiPolynomial(0)) { return aPoly; }
    auto aPolyDegree = aPoly.degree();
    auto coeff = aPoly.self[aPolyDegree];
    for (auto& iter : aPoly.self)
    {
      iter.second = iter.second / coeff;
    }
    return aPoly;
  }

  PiPolynomial PiPolynomial::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    PiPolynomial answer;
    answer.self[0] = 1;
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool PiPolynomial::operator==(const PiPolynomial& rhs) const
  {
    auto diff = (*this) - rhs;
    for (const auto& iter : diff.self)
    {
      if (iter.second != 0) { return false; }
    }
    return true;
  }

  bool PiPolynomial::operator!=(const PiPolynomial& rhs) const
  {
    return !((*this) == rhs);
  }

  bool PiPolynomial::operator<(const PiPolynomial& rhs) const
  {
    auto diff = (*this) - rhs;
    if (diff.re() == PiPolynomial(0))
    {
      return (diff.get().second < 0);
    }
    return (diff.get().first < 0);
  }

  PiPolynomial PiPolynomial::conjugate() const
  {
    auto conj = (*this);
    for (auto& iter : conj.self) { iter.second = iter.second.conjugate(); }
    return conj;
  }

  bool PiPolynomial::isReal() const { return ((*this) == conjugate()); }
  PiPolynomial PiPolynomial::re() const
  {
    auto answer = (*this) + conjugate();
    answer = answer * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return answer;
  }

  PiPolynomial PiPolynomial::im() const
  {
    auto answer = (*this) - conjugate();
    answer = answer * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return answer;
  }
}
