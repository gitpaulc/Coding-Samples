/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#include "rational_function.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  RationalFunction::RationalFunction(const AlgebraicPolynomial& nn, const AlgebraicPolynomial& dd)
  {
    if (dd == AlgebraicPolynomial(PiPolynomial(0)))
    {
      throw std::invalid_argument("Division by zero.");
    }
    else // TODO: Implement simplifying rational RationalFunctions later.
    {
      //auto gcd_ = AlgebraicPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      /*if (gcd_ != AlgebraicPolynomial(PiPolynomial(0)))
      {
        AlgebraicPolynomial remainder;
        num = num.division(gcd_, remainder);
        denom = denom.division(gcd_, remainder);
      }*/
    }
  }

  unsigned int RationalFunction::getDimension() const
  {
    auto dimNum = num.getDimension();
    auto dimDen = denom.getDimension();
    return (dimNum > dimDen) ? dimNum : dimDen;
  }

  std::string RationalFunction::print(bool useParentheses) const
  {
    auto num_ = num;
    auto den_ = denom;

    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    strm << "[";
    strm << num_.print(false);
    strm << "]";
    if ((den_ != AlgebraicPolynomial(PiPolynomial(1))) && (num_ != AlgebraicPolynomial(PiPolynomial(0))))
    {
      strm << " / ";
      strm << "[";
      strm << den_.print(false);
      strm << "]";
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  AlgebraicPolynomial RationalFunction::denominator() const { return denom; }
  AlgebraicPolynomial RationalFunction::numerator() const { return num; }

  RationalFunction RationalFunction::operator+() const
  {
    return *this;
  }

  RationalFunction RationalFunction::operator-() const
  {
    return RationalFunction(-num, denom);
  }

  RationalFunction RationalFunction::operator+(const RationalFunction& rhs) const
  {
    return RationalFunction(num * rhs.denom + rhs.num * denom, denom * rhs.denom);
  }

  RationalFunction RationalFunction::operator-(const RationalFunction& rhs) const
  {
    return ((*this) + (-rhs));
  }

  RationalFunction RationalFunction::operator*(const RationalFunction& rhs) const
  {
    return RationalFunction(num * rhs.num, denom * rhs.denom);
  }

  RationalFunction RationalFunction::operator/(const RationalFunction& rhs) const
  {
    if (rhs.num == AlgebraicPolynomial(PiPolynomial(0)))
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return RationalFunction(num * rhs.denom, denom * rhs.num);
  }

  RationalFunction RationalFunction::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    RationalFunction answer(AlgebraicPolynomial(PiPolynomial(1)), AlgebraicPolynomial(PiPolynomial(1)));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return RationalFunction(answer.denom, answer.num);
    }
    return answer;
  }

  bool RationalFunction::operator==(const RationalFunction& rhs) const
  {
    if (rhs.num != num) { return false; }
    if (rhs.denom != denom) { return false; }
    return true;
  }

  bool RationalFunction::operator!=(const RationalFunction& rhs) const
  {
    if (*this == rhs) { return false; }
    return true;
  }

  RationalFunction RationalFunction::constant(const PiRational& coeff)
  {
    AlgebraicPolynomial one(PiRational(PiPolynomial(ComplexQuadratic(1))));
    return RationalFunction(AlgebraicPolynomial(coeff), one);
  }

  RationalFunction RationalFunction::partial_deriv(unsigned int index) const
  {
    RationalFunction answer;
    auto numPrime = num.partial_deriv(index);
    auto denPrime = denom.partial_deriv(index);
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  RationalFunction RationalFunction::partial_x() const
  {
    return partial_deriv(0);
  }

  RationalFunction RationalFunction::partial_y() const
  {
    return partial_deriv(1);
  }

  RationalFunction RationalFunction::partial_z() const
  {
    return partial_deriv(2);
  }

  RationalFunction RationalFunction::partial_w() const
  {
    return partial_deriv(3);
  }

  RationalFunction RationalFunction::laplacian() const
  {
    RationalFunction answer;
    int dim = (int)getDimension();
    for (int ii = 0; ii < dim; ++ii)
    {
      answer = answer + (*this).partial_deriv(ii).partial_deriv(ii);
    }
    return answer;
  }

  bool RationalFunction::isLaplaceEigenfunction(PiRational& eigenvalue) const
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

  bool RationalFunction::isHarmonic() const
  {
    return (laplacian().num == AlgebraicPolynomial(PiPolynomial(0)));
  }

  RationalFunction RationalFunction::evaluateAt(const std::vector<PiRational>& input) const
  {
    auto numEval = num.evaluateAt(input);
    auto denEval = denom.evaluateAt(input);
    return RationalFunction(numEval, denEval);
  }

  RationalFunction RationalFunction::evaluateAt(const std::map<unsigned int, PiRational>& input) const
  {
    auto numEval = num.evaluateAt(input);
    auto denEval = denom.evaluateAt(input);
    return RationalFunction(numEval, denEval);
  }

  bool RationalFunction::tryEvaluate(const std::vector<PiRational>& input, PiRational& output) const
  {
    RationalFunction answer = evaluateAt(input);
    PiRational constAnsNum, constAnsDen;
    bool itIsConst = answer.num.isConstant(&constAnsNum);
    itIsConst = answer.denom.isConstant(&constAnsDen);
    if (!itIsConst) { return false; }
    output = constAnsNum / constAnsDen;
    return true;
  }
}
