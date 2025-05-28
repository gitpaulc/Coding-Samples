
#include "function.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  Function::Function(const FnPolynomial& nn, const FnPolynomial& dd)
  {
    if (dd == FnPolynomial(PiPolynomial(0)))
    {
      throw std::invalid_argument("Division by zero.");
    }
    else
    {
      auto gcd_ = FnPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      if (gcd_ != FnPolynomial(PiPolynomial(0)))
      {
        FnPolynomial remainder;
        num = num.division(gcd_, remainder);
        denom = denom.division(gcd_, remainder);
      }
    }
  }

  std::pair<double, double> Function::get() const
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

  std::string Function::print(bool useParentheses) const
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
    strm << num_.print(true);
    if (den_ != FnPolynomial(1))
    {
      strm << " / ";
      strm << den_.print(true);
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  FnPolynomial Function::denominator() const { return denom; }
  FnPolynomial Function::numerator() const { return num; }

  Function Function::operator+() const
  {
    return *this;
  }

  Function Function::operator-() const
  {
    return Function(-num, denom);
  }

  Function Function::operator+(const Function& rhs) const
  {
    return Function(num * rhs.denom + rhs.num * denom, denom * rhs.denom);
  }

  Function Function::operator-(const Function& rhs) const
  {
    return ((*this) + (-rhs));
  }

  Function Function::operator*(const Function& rhs) const
  {
    return Function(num * rhs.num, denom * rhs.denom);
  }

  Function Function::operator/(const Function& rhs) const
  {
    if (rhs.num == FnPolynomial(0))
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return Function(num * rhs.denom, denom * rhs.num);
  }

  Function Function::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    Function answer(FnPolynomial(1), FnPolynomial(1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return Function(answer.denom, answer.num);
    }
    return answer;
  }

  bool Function::operator==(const Function& rhs) const
  {
    if (rhs.num != num) { return false; }
    if (rhs.denom != denom) { return false; }
    return true;
  }

  bool Function::operator!=(const Function& rhs) const
  {
    if (*this == rhs) { return false; }
    return true;
  }

  bool Function::operator<(const Function& rhs) const
  {
    if (num * rhs.denom < denom * rhs.num) { return true; }
    return false;
  }

  bool Function::operator>(const Function& rhs) const
  {
    if (denom * rhs.num < num * rhs.denom) { return true; }
    return false;
  }

  bool Function::operator<=(const Function& rhs) const
  {
    if ((*this) == rhs) { return true; }
    if ((*this) < rhs) { return true; }
    return false;
  }

  bool Function::operator>=(const Function& rhs) const
  {
    if ((*this) == rhs) { return true; }
    if ((*this) > rhs) { return true; }
    return false;
  }
}
