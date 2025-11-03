/*  Copyright Paul Cernea, May 2025.
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

  RationalFunction RationalFunction::composeWith(const Matrix<ComplexQuadratic>& transform) const
  {
    RationalFunction answer;
    answer.num = num.composeWith(transform);
    answer.denom = denom.composeWith(transform);
    return answer;
  }

  RationalFunction RationalFunction::tanATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return RationalFunction(AlgebraicPolynomial::sinATimesPiX(coeff, A), AlgebraicPolynomial::cosATimesPiX(PiPolynomial(1), A));
  }

  RationalFunction RationalFunction::tanATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return RationalFunction(AlgebraicPolynomial::sinATimesPiY(coeff, A), AlgebraicPolynomial::cosATimesPiY(PiPolynomial(1), A));
  }

  RationalFunction RationalFunction::tanATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return RationalFunction(AlgebraicPolynomial::sinATimesPiZ(coeff, A), AlgebraicPolynomial::cosATimesPiZ(PiPolynomial(1), A));
  }

  RationalFunction RationalFunction::tanPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    return RationalFunction(AlgebraicPolynomial::sinPi_AX_plus_BY_plus_CZ(coeff, A, B, C),
      AlgebraicPolynomial::cosPi_AX_plus_BY_plus_CZ(PiPolynomial(1), A, B, C));
  }

  RationalFunction RationalFunction::partial_x() const
  {
    RationalFunction answer;
    auto numPrime = num.partial_x();
    auto denPrime = denom.partial_x();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  RationalFunction RationalFunction::partial_y() const
  {
    RationalFunction answer;
    auto numPrime = num.partial_y();
    auto denPrime = denom.partial_y();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  RationalFunction RationalFunction::partial_z() const
  {
    RationalFunction answer;
    auto numPrime = num.partial_z();
    auto denPrime = denom.partial_z();
    answer.num = numPrime * denom - num * denPrime;
    answer.denom = denom * denom;
    return answer;
  }

  RationalFunction RationalFunction::laplacian() const
  {
    auto u_x = num.partial_x(); auto u_y = num.partial_y(); auto u_z = num.partial_z();
    auto v_x = denom.partial_x(); auto v_y = denom.partial_y(); auto v_z = denom.partial_z();
    auto u_xx = u_x.partial_x(); auto u_yy = u_y.partial_y(); auto u_zz = u_z.partial_z();
    auto v_xx = v_x.partial_x(); auto v_yy = v_y.partial_y(); auto v_zz = v_z.partial_z();

    auto u_twice = num * AlgebraicPolynomial(PiPolynomial(2));
    auto v_twice = denom * AlgebraicPolynomial(PiPolynomial(2));
    auto v2 = denom * denom;
    auto v3 = denom * v2;
    auto uv = num * denom;

    auto xPortion = u_xx * v2 - v_xx * uv - u_x * v_x * v_twice + v_x * v_x * u_twice;
    auto yPortion = u_yy * v2 - v_yy * uv - u_y * v_y * v_twice + v_y * v_y * u_twice;
    auto zPortion = u_zz * v2 - v_zz * uv - u_z * v_z * v_twice + v_z * v_z * u_twice;

    return RationalFunction(xPortion + yPortion + zPortion, v3);
  }

  bool RationalFunction::isLaplaceEigenRationalFunction(PiRational& eigenvalue) const
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

  RationalFunction RationalFunction::sphericalBesselATimesPiX(const ComplexQuadratic& A, int n)
  {
    if (n < 0)
    {
      ComplexQuadratic qq(Rational(1, 1));
      if ((n % 2) == 1) { qq = -qq; }
      return sphericalNeumannATimesPiX(A, -n - 1) * AlgebraicPolynomial(PiPolynomial(qq));
    }
    if (n == 0)
    {
      PiPolynomial coeff = ComplexQuadratic(Rational(1, 1));
      PiRational piRatio = PiPolynomial(A, 1);
      return RationalFunction(AlgebraicPolynomial::sinATimesPiX(coeff, A), AlgebraicPolynomial::xToPower(piRatio, 1));
    }
    ComplexQuadratic qq(Rational(-1, 1));
    qq = qq / A;
    return sphericalBesselATimesPiX(A, n - 1).partial_x() * AlgebraicPolynomial(PiPolynomial(qq));
  }

  RationalFunction RationalFunction::sphericalNeumannATimesPiX(const ComplexQuadratic& A, int n)
  {
    if (n < 0)
    {
      ComplexQuadratic qq(Rational(1, 1));
      if ((n % 2) == 0) { qq = -qq; }
      return sphericalBesselATimesPiX(A, -n - 1) * AlgebraicPolynomial(PiPolynomial(qq));
    }
    if (n == 0)
    {
      PiPolynomial coeff = ComplexQuadratic(Rational(1, 1));
      PiRational piRatio = PiPolynomial(-A, 1);
      return RationalFunction(AlgebraicPolynomial::cosATimesPiX(coeff, A), AlgebraicPolynomial::xToPower(piRatio, 1));
    }
    ComplexQuadratic qq(Rational(-1, 1));
    qq = qq / A;
    return sphericalNeumannATimesPiX(A, n - 1).partial_x() * AlgebraicPolynomial(PiPolynomial(qq));
  }

  bool RationalFunction::tryEvaluateAtX(const ComplexQuadratic& xVal, RationalFunction& output) const
  {
    AlgebraicPolynomial numFn;
    bool success = num.tryEvaluateAtX(xVal, numFn);
    if (!success) { return false; }
    AlgebraicPolynomial denomFn;
    success = denom.tryEvaluateAtX(xVal, denomFn);
    if (!success) { return false; }
    output = RationalFunction(numFn, denomFn);
    return true;
  }

  bool RationalFunction::tryEvaluateAtY(const ComplexQuadratic& yVal, RationalFunction& output) const
  {
    AlgebraicPolynomial numFn;
    bool success = num.tryEvaluateAtY(yVal, numFn);
    if (!success) { return false; }
    AlgebraicPolynomial denomFn;
    success = denom.tryEvaluateAtY(yVal, denomFn);
    if (!success) { return false; }
    output = RationalFunction(numFn, denomFn);
    return true;
  }

  bool RationalFunction::tryEvaluateAtZ(const ComplexQuadratic& zVal, RationalFunction& output) const
  {
    AlgebraicPolynomial numFn;
    bool success = num.tryEvaluateAtZ(zVal, numFn);
    if (!success) { return false; }
    AlgebraicPolynomial denomFn;
    success = denom.tryEvaluateAtZ(zVal, denomFn);
    if (!success) { return false; }
    output = RationalFunction(numFn, denomFn);
    return true;
  }

  bool RationalFunction::tryEvaluateAtXYZ(const ComplexQuadratic& xVal, const ComplexQuadratic& yVal, const ComplexQuadratic& zVal, PiRational& output) const
  {
    PiRational numConstant;
    bool success = num.tryEvaluateAtXYZ(xVal, yVal, zVal, numConstant);
    if (!success) { return false; }
    PiRational denomConstant;
    success = denom.tryEvaluateAtXYZ(xVal, yVal, zVal, denomConstant);
    if (!success) { return false; }
    output = numConstant / denomConstant;
    return true;
  }
}
