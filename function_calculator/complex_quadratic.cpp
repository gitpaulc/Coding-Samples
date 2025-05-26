/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "complex_quadratic.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  ComplexQuadratic::ComplexQuadratic(const QuadraticNumber& reIn, const QuadraticNumber& imIn)
  {
    re = reIn; im = imIn;
  }

  std::pair<double, double> ComplexQuadratic::get() const
  {
    return { re.get().first, im.get().first };
  }

  std::string ComplexQuadratic::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    bool reIsZero = (re == Rational(0));
    bool imIsZero = (im == Rational(0));
    if (!reIsZero)
    {
      strm << re.print(true);
      if (!imIsZero) { strm << " + "; }
    }
    if (!imIsZero)
    {
      strm << im.print(true);
      strm << " * i";
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  ComplexQuadratic ComplexQuadratic::conjugate() const { return ComplexQuadratic(re, -im); }

  QuadraticNumber ComplexQuadratic::sqLength() const
  {
    return re * re + im * im;
  }

  ComplexQuadratic ComplexQuadratic::sqrt(const Rational& radicand)
  {
    ComplexQuadratic answer;
    if (radicand < 0) { answer.im = QuadraticNumber::sqrt(-radicand); }
    else { answer.re = QuadraticNumber::sqrt(radicand); }
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::operator+() const
  {
    return *this;
  }

  ComplexQuadratic ComplexQuadratic::operator-() const
  {
    auto answer = *this;
    answer.re = -answer.re; answer.im = -answer.im;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::operator+(const ComplexQuadratic& rhs) const
  {
    auto answer = *this;
    answer.re = answer.re + rhs.re;
    answer.im = answer.im + rhs.im;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::operator-(const ComplexQuadratic& rhs) const
  {
    auto answer = *this;
    answer.re = answer.re - rhs.re;
    answer.im = answer.im - rhs.im;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::operator*(const ComplexQuadratic& rhs) const
  {
    auto answer = *this;
    answer.re = re * rhs.re - im * rhs.im;
    answer.im = re * rhs.im + im * rhs.re;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::operator/(const ComplexQuadratic& rhs) const
  {
    auto answer = *this;
    auto conj = rhs;
    conj.im = -conj.im;
    answer = answer * conj;
    auto sq_norm = sqLength();
    answer.re = answer.re / sq_norm;
    answer.im = answer.im / sq_norm;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    ComplexQuadratic answer(Rational(1, 1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return ComplexQuadratic(Rational(1, 1)) / answer;
    }
    return answer;
  }

  bool ComplexQuadratic::operator==(const ComplexQuadratic& rhs) const
  {
    if (re != rhs.re) { return false; }
    if (im != rhs.im) { return false; }
    return true;
  }

  bool ComplexQuadratic::operator!=(const ComplexQuadratic& rhs) const
  {
    return !((*this) == rhs);
  }
}
