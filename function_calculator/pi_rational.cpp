/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "pi_rational.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  PiRational::PiRational(const PiPolynomial& nn, const PiPolynomial& dd)
  {
    if (dd == PiPolynomial(0))
    {
      throw std::invalid_argument("Division by zero.");
    }
    else
    {
      auto gcd_ = PiPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      if (gcd_ != PiPolynomial(0))
      {
        PiPolynomial remainder;
        num = num.division(gcd_, remainder);
        denom = denom.division(gcd_, remainder);
      }
    }
  }

  std::pair<double, double> PiRational::get() const
  {
    if (denom.isReal())
    {
      auto numerGet = num.get();
      auto toDivide = denom.get().first;
      numerGet.first /= toDivide;
      numerGet.second /= toDivide;
      return numerGet;
    }
    auto denomConj = denom.conjugate();
    auto numer = (num * denomConj).get();
    auto toDivide = (denom * denomConj).get().first;
    numer.first /= toDivide;
    numer.second /= toDivide;
    return numer;
  }

  std::string PiRational::print(bool useParentheses) const
  {
    auto num_ = num;
    auto den_ = denom;

    if (!denom.isReal())
    {
      auto denomConj = denom.conjugate();
      num_ = num_ * denomConj;
      den_ = den_ * denomConj;
    }

    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    strm << num_.print(true) << " / " << den_.print(true);
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  PiPolynomial PiRational::denominator() const { return denom; }
  PiPolynomial PiRational::numerator() const { return num; }

  PiRational PiRational::operator+() const
  {
    return *this;
  }

  PiRational PiRational::operator-() const
  {
    return PiRational(-num, denom);
  }

  PiRational PiRational::operator+(const PiRational& rhs) const
  {
    return PiRational(num * rhs.denom + rhs.num * denom, denom * rhs.denom);
  }

  PiRational PiRational::operator-(const PiRational& rhs) const
  {
    return ((*this) + (-rhs));
  }

  PiRational PiRational::operator*(const PiRational& rhs) const
  {
    return PiRational(num * rhs.num, denom * rhs.denom);
  }

  PiRational PiRational::operator/(const PiRational& rhs) const
  {
    if (rhs.num == PiPolynomial(0))
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return PiRational(num * rhs.denom, denom * rhs.num);
  }

  PiRational PiRational::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    PiRational answer(PiPolynomial(1), PiPolynomial(1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return PiRational(answer.denom, answer.num);
    }
    return answer;
  }

  bool PiRational::operator==(const PiRational& rhs) const
  {
    if (rhs.num != num) { return false; }
    if (rhs.denom != denom) { return false; }
    return true;
  }

  bool PiRational::operator!=(const PiRational& rhs) const
  {
    if (*this == rhs) { return false; }
    return true;
  }

  bool PiRational::operator<(const PiRational& rhs) const
  {
    if (num * rhs.denom < denom * rhs.num) { return true; }
    return false;
  }

  bool PiRational::operator>(const PiRational& rhs) const
  {
    if (denom * rhs.num < num * rhs.denom) { return true; }
    return false;
  }

  bool PiRational::operator<=(const PiRational& rhs) const
  {
    if ((*this) == rhs) { return true; }
    if ((*this) < rhs) { return true; }
    return false;
  }

  bool PiRational::operator>=(const PiRational& rhs) const
  {
    if ((*this) == rhs) { return true; }
    if ((*this) > rhs) { return true; }
    return false;
  }
}
