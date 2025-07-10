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

  ComplexQuadratic::ComplexQuadratic(const BiquadraticNumber& reIn, const BiquadraticNumber& imIn)
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
    auto answer = strm.str();
    if (answer.empty()) { answer = "0"; }
    trimParentheses(answer, { '(', ')' });
    if (useParentheses) { answer = std::string("(") + answer + ")"; }
    return answer;
  }

  BiquadraticNumber ComplexQuadratic::getRe() const { return re; }
  BiquadraticNumber ComplexQuadratic::getIm() const { return im; }

  ComplexQuadratic ComplexQuadratic::conjugate() const { return ComplexQuadratic(re, -im); }
  bool ComplexQuadratic::isReal() const { return (*this) == conjugate(); }

  BiquadraticNumber ComplexQuadratic::sqLength() const
  {
    return re * re + im * im;
  }

  ComplexQuadratic ComplexQuadratic::sqrt(const Rational& radicand)
  {
    ComplexQuadratic answer;
    if (radicand < 0) { answer.im = BiquadraticNumber::sqrt(-radicand); }
    else { answer.re = BiquadraticNumber::sqrt(radicand); }
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::sqrtOfITimes(const Rational& radicand)
  {
    auto sqrtOfI = ComplexQuadratic(BiquadraticNumber::sqrt(Rational(1, 2)), BiquadraticNumber::sqrt(Rational(1, 2)));
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

  namespace // anonymous
  {
    std::map<std::pair<ComplexQuadratic, int>, ComplexQuadratic> powerCache;
  }

  ComplexQuadratic ComplexQuadratic::pow(int p) const
  {
    if (p == 0) { return ComplexQuadratic(Rational(1, 1)); }
    if (p < 0) { return ComplexQuadratic(Rational(1, 1)) / pow(-p); }
    {
      auto iter = powerCache.find({*this, p});
      if (iter != powerCache.end())
      {
        return iter->second;
      }
    }
    auto p1 = p / 2;
    auto p2 = p - p1;
    if ((p1 < p) && (p2 < p))
    {
      return pow(p1) * pow(p2);
    }
    ComplexQuadratic answer(Rational(1, 1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    powerCache[{*this, p}] = answer;
    return answer;
  }

  ComplexQuadratic ComplexQuadratic::aPlusBTotheP(const ComplexQuadratic& aa, const ComplexQuadratic& bb, unsigned int p)
  {
    std::vector<ComplexQuadratic> aaPowers(p + 1);
    std::vector<ComplexQuadratic> bbPowers(p + 1);
    aaPowers[0] = ComplexQuadratic(1);
    bbPowers[0] = ComplexQuadratic(1);
    int pp = (int)p;
    for (int ii = 0; ii < pp; ++ii)
    {
      aaPowers[ii + 1] = aaPowers[ii] * aa;
      bbPowers[ii + 1] = bbPowers[ii] * bb;
    }
    ComplexQuadratic sum;
    for (int ii = 0; ii <= pp; ++ii)
    {
      sum = sum + aaPowers[ii] * (bbPowers[p - ii] * BiquadraticNumber(Rational(mp::binomialCoeff((int)p, ii), 1)));
    }
    return sum;
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
