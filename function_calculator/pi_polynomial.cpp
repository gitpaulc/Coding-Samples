/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "pi_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  void PiPolynomial::clean()
  {
    int lastNonzero = -1;
    for (int i = (int)self.size() - 1; i >= 0; i--)
    {
      if (self[i] == ComplexQuadratic()) { continue; }
      lastNonzero = i;
      break;
    }
    self.resize(lastNonzero + 1);
  }

  PiPolynomial::PiPolynomial(const ComplexQuadratic& coeff, int power)
  {
    if (power < 0) { throw std::invalid_argument("Exponent must be nonnegative."); }
    else
    {
      self.resize(power + 1);
      self[power] = coeff;
      clean();
    }
  }

  std::pair<double, double> PiPolynomial::get() const
  {
    double answerRe = 0.0;
    double answerIm = 0.0;
    for (int i = 0; i < (int)self.size(); ++i)
    {
      answerRe *= piValue();
      answerIm *= piValue();
      answerRe += self[i].getRe().get().first;
      answerIm += self[i].getIm().get().first;
    }
    return { answerRe, answerIm };
  }

  std::string PiPolynomial::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    int count = -1;
    for (int i = 0; i < (int)self.size(); ++i)
    {
      if (self[i] == ComplexQuadratic()) { continue; }
      ++count;
      if (count != 0) { strm << " + "; }
      strm << self[i].print(true);
      if (i == 1) { strm << "Pi"; }
      else { strm << "(Pi)"; }
      if (i > 1) { strm << "^" << i; }
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
    for (int i = 0; i < (int)self.size(); ++i)
    {
      answer.self[i] = -answer.self[i];
    }
    return answer;
  }

  PiPolynomial PiPolynomial::operator+(const PiPolynomial& rhs) const
  {
    if (rhs.self.size() > self.size()) { return rhs + (*this); }

    auto answer = *this;
    for (int i = 0; i < (int)self.size(); ++i)
    {
      ComplexQuadratic summand;
      if (i < (int)rhs.self.size()) { summand = rhs.self[i]; }
      answer.self[i] = answer.self[i] + summand;
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
    auto answer = *this;
    answer.self.resize(self.size() + rhs.self.size());

    for (int i = 0; i < (int)self.size(); ++i)
    {
      for (int j = 0; j < (int)rhs.self.size(); ++j)
      {
        answer.self[i + j] = answer.self[i + j] + self[i] + rhs.self[i];
      }
    }
    return answer;
  }

  PiPolynomial PiPolynomial::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    PiPolynomial answer;
    answer.self.resize(1);
    answer.self[0] = ComplexQuadratic(QuadraticNumber(Rational(1, 1)));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool PiPolynomial::operator==(const PiPolynomial& rhs) const
  {
    for (int i = 0; i < (int)self.size(); ++i)
    {
      if (self[i] != rhs.self[i]) { return false; }
    }
    return true;
  }

  bool PiPolynomial::operator!=(const PiPolynomial& rhs) const
  {
    return !((*this) == rhs);
  }
}
