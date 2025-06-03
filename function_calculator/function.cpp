
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
      //auto gcd_ = FnPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      /*if (gcd_ != FnPolynomial(PiPolynomial(0)))
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
    strm << "[";
    strm << num_.print(false);
    strm << "]";
    if ((den_ != FnPolynomial(PiPolynomial(1))) && (num_ != FnPolynomial(PiPolynomial(0))))
    {
      strm << " / ";
      strm << "[";
      strm << den_.print(false);
      strm << "]";
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

  Function Function::constant(const PiRational& coeff)
  {
    FnPolynomial one(PiRational(PiPolynomial(ComplexQuadratic(1))));
    return Function(FnPolynomial(coeff), one);
  }

  Function Function::tanATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return Function(FnPolynomial::sinATimesPiX(coeff, A), FnPolynomial::cosATimesPiX(PiPolynomial(1), A));
  }

  Function Function::tanATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return Function(FnPolynomial::sinATimesPiY(coeff, A), FnPolynomial::cosATimesPiY(PiPolynomial(1), A));
  }

  Function Function::tanATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    return Function(FnPolynomial::sinATimesPiZ(coeff, A), FnPolynomial::cosATimesPiZ(PiPolynomial(1), A));
  }

  Function Function::tanPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    return Function(FnPolynomial::sinPi_AX_plus_BY_plus_CZ(coeff, A, B, C),
      FnPolynomial::cosPi_AX_plus_BY_plus_CZ(PiPolynomial(1), A, B, C));
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
    auto u_x = num.partial_x(); auto u_y = num.partial_y(); auto u_z = num.partial_z();
    auto v_x = denom.partial_x(); auto v_y = denom.partial_y(); auto v_z = denom.partial_z();
    auto u_xx = u_x.partial_x(); auto u_yy = u_y.partial_y(); auto u_zz = u_z.partial_z();
    auto v_xx = v_x.partial_x(); auto v_yy = v_y.partial_y(); auto v_zz = v_z.partial_z();

    auto u_twice = num * FnPolynomial(PiPolynomial(2));
    auto v_twice = denom * FnPolynomial(PiPolynomial(2));
    auto v2 = denom * denom;
    auto v3 = denom * v2;
    auto uv = num * denom;

    auto xPortion = u_xx * v2 - v_xx * uv - u_x * v_x * v_twice + v_x * v_x * u_twice;
    auto yPortion = u_yy * v2 - v_yy * uv - u_y * v_y * v_twice + v_y * v_y * u_twice;
    auto zPortion = u_zz * v2 - v_zz * uv - u_z * v_z * v_twice + v_z * v_z * u_twice;

    return Function(xPortion + yPortion + zPortion, v3);
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
    return (laplacian().num == FnPolynomial(PiPolynomial(0)));
  }

  Function Function::sphericalBesselATimesPiX(const ComplexQuadratic& A, int n)
  {
    if (n < 0)
    {
      ComplexQuadratic qq(Rational(1, 1));
      if ((n % 2) == 1) { qq = -qq; }
      return sphericalNeumannATimesPiX(A, -n - 1) * FnPolynomial(PiPolynomial(qq));
    }
    if (n == 0)
    {
      PiPolynomial coeff = ComplexQuadratic(Rational(1, 1));
      PiRational piRatio = PiPolynomial(A, 1);
      return Function(FnPolynomial::sinATimesPiX(coeff, A), FnPolynomial::xToPower(piRatio, 1));
    }
    ComplexQuadratic qq(Rational(-1, 1));
    qq = qq / A;
    return sphericalBesselATimesPiX(A, n - 1).partial_x() * FnPolynomial(PiPolynomial(qq));
  }

  Function Function::sphericalNeumannATimesPiX(const ComplexQuadratic& A, int n)
  {
    if (n < 0)
    {
      ComplexQuadratic qq(Rational(1, 1));
      if ((n % 2) == 0) { qq = -qq; }
      return sphericalBesselATimesPiX(A, -n - 1) * FnPolynomial(PiPolynomial(qq));
    }
    if (n == 0)
    {
      PiPolynomial coeff = ComplexQuadratic(Rational(1, 1));
      PiRational piRatio = PiPolynomial(-A, 1);
      return Function(FnPolynomial::cosATimesPiX(coeff, A), FnPolynomial::xToPower(piRatio, 1));
    }
    ComplexQuadratic qq(Rational(-1, 1));
    qq = qq / A;
    return sphericalNeumannATimesPiX(A, n - 1).partial_x() * FnPolynomial(PiPolynomial(qq));
  }
}
