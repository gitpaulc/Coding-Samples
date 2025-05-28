
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
    else // TODO: Implement simplifying rational functions later.
    {
        /*
      auto gcd_ = FnPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      if (gcd_ != FnPolynomial(PiPolynomial(0)))
      {
        FnPolynomial remainder;
        num = num.division(gcd_, remainder);
        denom = denom.division(gcd_, remainder);
      }*/
    }
  }

  std::string Function::print(bool useParentheses) const
  {
    auto num_ = num;
    auto den_ = denom;

    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    strm << num_.print(true);
    if (den_ != FnPolynomial(PiPolynomial(1)))
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
    if (rhs.num == FnPolynomial(PiPolynomial(0)))
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return Function(num * rhs.denom, denom * rhs.num);
  }

  Function Function::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    Function answer(FnPolynomial(PiPolynomial(1)), FnPolynomial(PiPolynomial(1)));
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

  Function Function::partial_x() const
  {
    Function answer;
    auto numPrime = num.partial_x();
    auto denPrime = denom.partial_x();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  Function Function::partial_y() const
  {
    Function answer;
    auto numPrime = num.partial_y();
    auto denPrime = denom.partial_y();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  Function Function::partial_z() const
  {
    Function answer;
    auto numPrime = num.partial_z();
    auto denPrime = denom.partial_z();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  Function Function::laplacian() const
  {
    Function answer = (*this).partial_x().partial_x();
    answer = answer + (*this).partial_y().partial_y();
    answer = answer + (*this).partial_z().partial_z();
    return answer;
  }

  bool Function::isLaplaceEigenfunction(PiRational& eigenvalue) const
  {
    if (isHarmonic()) { eigenvalue = PiRational(PiPolynomial(0), PiPolynomial(1)); return true; }
    auto lap0 = laplacian();
    auto lap = lap0.num * (*this).denom;
    auto original = lap0.denom * (*this).num;
    PiRational eigen;
    for (const auto& iter : original.self)
    {
      if (lap.self.find(iter.first) == lap.self.end()) { return false; }
      auto nn = lap.self.at(iter.first);
      auto dd = original.self.at(iter.first);
      if (dd == PiRational(PiPolynomial(0), PiPolynomial(1))) { return false; } // We would have already detected harmonic.
      eigen = -nn / dd;
      break;
    }
    auto comparer = original * (-eigen);
    bool answer = (lap == comparer);
    if (answer) { eigenvalue = eigen; }
    return answer;
  }

  bool Function::isHarmonic() const
  {
    auto zero = Function();
    return (laplacian() == zero);
  }
}
