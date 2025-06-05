/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "complex_quadratic.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  ComplexQuadratic::ComplexQuadratic(int reIn)
  {
    re = Rational(reIn, 1); im = Rational(0, 1);
  }

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
    bool imIsOne = (im == Rational(1));
    bool imIsNegOne = (im == Rational(-1));
    if (!reIsZero)
    {
      strm << re.print(true);
      if (!imIsZero)
      {
        if (imIsNegOne)
        {
          strm << " - ";
          imIsNegOne = false;
          imIsOne = true;
        }
        else { strm << " + "; }
      }
    }
    if (!imIsZero)
    {
      if (!imIsOne && !imIsNegOne)
      {
        strm << im.print(true);
        strm << " * ";
      }
      else if (imIsNegOne) { strm << "-"; }
      strm << "i";
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  QuadraticNumber ComplexQuadratic::getRe() const { return re; }
  QuadraticNumber ComplexQuadratic::getIm() const { return im; }

  ComplexQuadratic ComplexQuadratic::conjugate() const { return ComplexQuadratic(re, -im); }
  bool ComplexQuadratic::isReal() const { return (*this) == conjugate(); }

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

  ComplexQuadratic ComplexQuadratic::sqrtOfITimes(const Rational& radicand)
  {
    auto sqrtOfI = ComplexQuadratic(QuadraticNumber::sqrt(Rational(1, 2)), QuadraticNumber::sqrt(Rational(1, 2)));
    return sqrtOfI * ComplexQuadratic::sqrt(radicand);
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
    auto sq_norm = rhs.sqLength();
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

  bool ComplexQuadratic::operator<(const ComplexQuadratic& rhs) const
  {
    if (re < rhs.re) { return true; }
    if (rhs.re < re) { return false; }
    return im < rhs.im;
  }

  bool ComplexQuadratic::operator>(const ComplexQuadratic& rhs) const
  {
    return (rhs < (*this));
  }
}
