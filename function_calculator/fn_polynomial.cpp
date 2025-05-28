/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "fn_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  bool FnPolynomial::Monomial::isConstTerm() const
  {
    if (xInd != 0) { return false; }
    if (yInd != 0) { return false; }
    if (zInd != 0) { return false; }
    if (ePiXInd != 0) { return false; }
    if (ePiYInd != 0) { return false; }
    if (ePiZInd != 0) { return false; }
    return true;
  }

  FnPolynomial::Monomial FnPolynomial::Monomial::operator+(const FnPolynomial::Monomial& rhs) const
  {
    Monomial answer;
    answer.xInd = xInd + rhs.xInd;
    answer.yInd = yInd + rhs.yInd;
    answer.zInd = zInd + rhs.zInd;
    answer.ePiXInd = ePiXInd + rhs.ePiXInd;
    answer.ePiYInd = ePiYInd + rhs.ePiYInd;
    answer.ePiZInd = ePiZInd + rhs.ePiZInd;
    return answer;
  }

  bool FnPolynomial::Monomial::operator<(const FnPolynomial::Monomial& rhs) const
  {
    if (xInd < rhs.xInd) { return true; }
    if (xInd > rhs.xInd) { return false; }
    if (yInd < rhs.yInd) { return true; }
    if (yInd > rhs.yInd) { return false; }
    if (zInd < rhs.zInd) { return true; }
    if (zInd > rhs.zInd) { return false; }
    if (ePiXInd < rhs.ePiXInd) { return true; }
    if (ePiXInd > rhs.ePiXInd) { return false; }
    if (ePiYInd < rhs.ePiYInd) { return true; }
    if (ePiYInd > rhs.ePiYInd) { return false; }
    if (ePiZInd < rhs.ePiZInd) { return true; }
    if (ePiZInd > rhs.ePiZInd) { return false; }
    return false; // They are equal.
  }

  void FnPolynomial::clean()
  {
    FnPolynomial answer;
    for (const auto& iter : self)
    {
      if (iter.second == PiRational()) { continue; }
      answer.self[iter.first] = iter.second;
    }
    self = answer.self;
  }

  FnPolynomial::FnPolynomial(const PiRational& coeff)
  {
    if (coeff != PiRational())
    {
      Monomial constTerm;
      self[constTerm] = coeff;
    }
  }

  std::string FnPolynomial::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    bool useBrackets = true;
    if (self.size() == 1)
    {
      useBrackets = !(self.begin()->first.isConstTerm());
    }
    int count = -1;
    for (const auto& iter : self)
    {
      if (iter.second == PiPolynomial()) { continue; }
      ++count;
      if (count != 0) { strm << " + "; }
      strm << iter.second.print(useBrackets);
      if (iter.first.isConstTerm()) { continue; }
      strm << " * ";
      if (iter.first.xInd != 0) { strm << "x^" << iter.first.xInd; }
      if (iter.first.yInd != 0) { strm << "y^" << iter.first.yInd; }
      if (iter.first.zInd != 0) { strm << "z^" << iter.first.zInd; }
      if (iter.first.ePiXInd != 0) { strm << "e^{Pi * " << iter.first.ePiXInd.print(true) << " * x}"; }
      if (iter.first.ePiYInd != 0) { strm << "e^{Pi * " << iter.first.ePiYInd.print(true) << " * y}"; }
      if (iter.first.ePiZInd != 0) { strm << "e^{Pi * " << iter.first.ePiZInd.print(true) << " * z}"; }
    }
    if (count < 0) { strm << "0"; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  FnPolynomial FnPolynomial::xToPower(const PiRational& coeff, int p)
  {
    Monomial term;
    term.xInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::yToPower(const PiRational& coeff, int p)
  {
    Monomial term;
    term.yInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::zToPower(const PiRational& coeff, int p)
  {
    Monomial term;
    term.zInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiXInd = A;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiYInd = A;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiZInd = A;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::eToThePi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(ComplexQuadratic(1));
    return eToTheATimesPiX(coeff, A) * eToTheATimesPiY(one, B) * eToTheATimesPiZ(one, C);
  }

  FnPolynomial FnPolynomial::sinATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiX(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiX(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::sinATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiY(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiY(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::sinATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiZ(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiZ(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::sinPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // sin(A)cos(B)cos(C) - sin(A)sin(B)sin(C)
    // + cos(A)sin(B)cos(C) + cos(A)cos(B)sin(C)
    return sinATimesPiX(coeff, A) * cosATimesPiY(one, B) * cosATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * sinATimesPiY(one, B) * sinATimesPiZ(one, C)
    + cosATimesPiX(coeff, A) * sinATimesPiY(one, B) * cosATimesPiZ(one, C)
    + cosATimesPiX(coeff, A) * cosATimesPiY(one, B) * sinATimesPiZ(one, C);
  }

  FnPolynomial FnPolynomial::cosATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiX(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiX(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::cosATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiY(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiY(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::cosATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiZ(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiZ(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  FnPolynomial FnPolynomial::cosPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // cos(A)cos(B)cos(C) - cos(A)sin(B)sin(C)
    // - sin(A)sin(B)cos(C) - sin(A)cos(B)sin(C)
    return cosATimesPiX(coeff, A) * cosATimesPiY(one, B) * cosATimesPiZ(one, C)
    - cosATimesPiX(coeff, A) * sinATimesPiY(one, B) * sinATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * sinATimesPiY(one, B) * cosATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * cosATimesPiY(one, B) * sinATimesPiZ(one, C);
  }

  FnPolynomial FnPolynomial::sinhATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiX(coeffNew, A) - eToTheATimesPiX(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::sinhATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiY(coeffNew, A) - eToTheATimesPiY(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::sinhATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiZ(coeffNew, A) - eToTheATimesPiZ(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::sinhPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // sinh(A)cosh(B)cosh(C) + sinh(A)sinh(B)sinh(C)
    // + cosh(A)sinh(B)cosh(C) + cosh(A)cosh(B)sinh(C)
    return sinhATimesPiX(coeff, A) * coshATimesPiY(one, B) * coshATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * sinhATimesPiY(one, B) * sinhATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * sinhATimesPiY(one, B) * coshATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * coshATimesPiY(one, B) * sinhATimesPiZ(one, C);
  }

  FnPolynomial FnPolynomial::coshATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiX(coeffNew, A) + eToTheATimesPiX(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::coshATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiY(coeffNew, A) + eToTheATimesPiY(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::coshATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiZ(coeffNew, A) + eToTheATimesPiZ(coeffNew, -A);
  }

  FnPolynomial FnPolynomial::coshPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // cosh(A)cosh(B)cosh(C) + cosh(A)sinh(B)sinh(C)
    // + sinh(A)sinh(B)cosh(C) + sinh(A)cosh(B)sinh(C)
    return coshATimesPiX(coeff, A) * coshATimesPiY(one, B) * coshATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * sinhATimesPiY(one, B) * sinhATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * sinhATimesPiY(one, B) * coshATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * coshATimesPiY(one, B) * sinhATimesPiZ(one, C);
  }

  FnPolynomial FnPolynomial::operator+() const
  {
    return *this;
  }

  FnPolynomial FnPolynomial::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.self)
    {
      iter.second = -iter.second;
    }
    return answer;
  }

  FnPolynomial FnPolynomial::operator+(const FnPolynomial& rhs) const
  {
    FnPolynomial answer = *this;

    for (auto& iter : rhs.self)
    {
      if (answer.self.find(iter.first) == answer.self.end())
      {
        answer.self[iter.first] = iter.second;
        continue;
      }
      answer.self[iter.first] = answer.self[iter.first] + iter.second;
    }
    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::operator-(const FnPolynomial& rhs) const
  {
    return (*this) + (-rhs);
  }

  FnPolynomial FnPolynomial::operator*(const FnPolynomial& rhs) const
  {
    FnPolynomial answer;

    for (const auto& iter : self)
    {
      for (const auto& jter : rhs.self)
      {
        auto summand = iter.second * jter.second;
        auto kk = iter.first + jter.first;
        if (answer.self.find(kk) == answer.self.end())
        {
          answer.self[kk] = summand;
          continue;
        }
        answer.self[kk] = answer.self[kk] + summand;
      }
    }
    return answer;
  }

  FnPolynomial FnPolynomial::operator*(const PiPolynomial& rhs) const
  {
    FnPolynomial answer;
    for (const auto& iter : self) { answer.self[iter.first] = iter.second * PiRational(PiPolynomial(rhs)); }
    return answer;
  }

  FnPolynomial FnPolynomial::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    FnPolynomial answer;
    Monomial constTerm;
    answer.self[constTerm] = PiPolynomial(ComplexQuadratic(1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool FnPolynomial::operator==(const FnPolynomial& rhs) const
  {
    auto diff = (*this) - rhs;
    for (const auto& iter : diff.self)
    {
      if (iter.second != PiPolynomial(0)) { return false; }
    }
    return true;
  }

  bool FnPolynomial::operator!=(const FnPolynomial& rhs) const
  {
    return !((*this) == rhs);
  }


  FnPolynomial FnPolynomial::partial_x() const
  {
    FnPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.xInd));
      --(newIndex.first.xInd);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(newIndex.first.ePiXInd);
      newIndex.second = newIndex.second * PiPolynomial(1, 1);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }
    }
    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::partial_y() const
  {
    FnPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.yInd));
      --(newIndex.first.yInd);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(newIndex.first.ePiYInd);
      newIndex.second = newIndex.second * PiPolynomial(1, 1);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }
    }
    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::partial_z() const
  {
    FnPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.zInd));
      --(newIndex.first.zInd);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(newIndex.first.ePiZInd);
      newIndex.second = newIndex.second * PiPolynomial(1, 1);
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }
    }
    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::laplacian() const
  {
    auto xPortion = (*this).partial_x().partial_x();
    auto yPortion = (*this).partial_y().partial_y();
    auto zPortion = (*this).partial_z().partial_z();
    return xPortion + yPortion + zPortion;
  }

  bool FnPolynomial::isLaplaceEigenfunction(PiRational& eigenvalue) const
  {
    if (isHarmonic()) { eigenvalue = PiRational(PiPolynomial(0), PiPolynomial(1)); return true; }
    auto lap = laplacian();
    PiRational eigen;
    for (const auto& iter : self)
    {
      if (lap.self.find(iter.first) == lap.self.end()) { return false; }
      auto nn = lap.self.at(iter.first);
      auto dd = self.at(iter.first);
      if (dd == PiRational(PiPolynomial(0), PiPolynomial(1))) { return false; } // We would have already detected harmonic.
      eigen = -nn / dd;
      break;
    }
    auto comparer = (*this) * (-eigen);
    bool answer = (lap == comparer);
    if (answer) { eigenvalue = eigen; }
    return answer;
  }

  bool FnPolynomial::isHarmonic() const
  {
    return (laplacian() == FnPolynomial(PiPolynomial(0)));
  }
}
