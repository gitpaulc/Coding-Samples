/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#include "algebraic_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  bool AlgebraicPolynomial::Monomial::isConstTerm() const
  {
    return indices.empty();
  }

  unsigned int AlgebraicPolynomial::Monomial::getDimension() const
  {
    unsigned int maxDim = 0;
    for (const auto& it : indices)
    {
      if (it.second == 0) { continue; }
      if ((it.first + 1) > maxDim) { maxDim = it.first + 1; }
    }
    return maxDim;
  }

  bool AlgebraicPolynomial::Monomial::operator<(const AlgebraicPolynomial::Monomial& rhs) const
  {
    auto dimLhs = getDimension();
    auto dimRhs = rhs.getDimension();
    if (dimLhs < dimRhs) { return true; }
    if (dimLhs > dimRhs) { return false; }
    // dimLhs equals dimRhs.
    std::vector<unsigned int> lhsVec(dimLhs, 0);
    std::vector<unsigned int> rhsVec(dimRhs, 0);
    for (const auto& it : indices)
    {
      lhsVec[it.first] = it.second;
    }
    for (const auto& it : rhs.indices)
    {
      rhsVec[it.first] = it.second;
    }
    for (int ii = dimLhs - 1; ii >= 0; --ii)
    {
      if (lhsVec[ii] < rhsVec[ii]) { return true; }
      if (lhsVec[ii] > rhsVec[ii]) { return false; }
    }
    return false; // They are equal.
  }

  void AlgebraicPolynomial::Monomial::clean()
  {
    Monomial cleaned;
    for (const auto& it : indices)
    {
      if (it.second != 0) { cleaned.indices[it.first] = it.second; }
    }
    *this = cleaned;
  }

  void AlgebraicPolynomial::clean()
  {
    AlgebraicPolynomial answer;

    for (const auto& iter : self)
    {
      if (iter.second == PiRational()) { continue; }
      {
        bool xNegative = false;
        bool yNegative = false;
        bool zNegative = false;
        auto jter = answer.trigFind(iter.first, xNegative, yNegative, zNegative);
        if (jter != answer.self.end())
        {
          if (jter->first.trigPiXInd.isCos()) { xNegative = false; }
          if (jter->first.trigPiYInd.isCos()) { yNegative = false; }
          if (jter->first.trigPiZInd.isCos()) { zNegative = false; }
          bool isNegative = false;
          if (xNegative) { isNegative = !isNegative; }
          if (yNegative) { isNegative = !isNegative; }
          if (zNegative) { isNegative = !isNegative; }
          if (isNegative) { jter->second = jter->second - iter.second; }
          else { jter->second = jter->second + iter.second; }
          continue;
        }
      }

      answer.self[iter.first] = iter.second;
    }
    self = std::map<Monomial, PiRational>();
    for (auto& iter : answer.self)
    {
      auto coeff = iter.second;
      auto newIndex = iter.first;
      if (iter.first.trigPiXInd != TrigIndex())
      {
        if (iter.first.trigPiXInd.self < BiquadraticNumber(0))
        {
          newIndex.trigPiXInd.self = -iter.first.trigPiXInd.self;
          if (!iter.first.trigPiXInd.isCosine) { coeff = -coeff; }
        }
      }
      if (iter.first.trigPiYInd != TrigIndex())
      {
        if (iter.first.trigPiYInd.self < BiquadraticNumber(0))
        {
          newIndex.trigPiYInd.self = -iter.first.trigPiYInd.self;
          if (!iter.first.trigPiYInd.isCosine) { coeff = -coeff; }
        }
      }
      if (iter.first.trigPiZInd != TrigIndex())
      {
        if (iter.first.trigPiZInd.self < BiquadraticNumber(0))
        {
          newIndex.trigPiZInd.self = -iter.first.trigPiZInd.self;
          if (!iter.first.trigPiZInd.isCosine) { coeff = -coeff; }
        }
      }
      self[newIndex] = coeff;
    }
  }

  AlgebraicPolynomial::AlgebraicPolynomial(const PiRational& coeff)
  {
    if (coeff != PiRational())
    {
      Monomial constTerm;
      self[constTerm] = coeff;
    }
  }

  std::string AlgebraicPolynomial::print(bool useParentheses) const
  {
    std::stringstream strm;
    bool useBrackets = true;
    if (self.size() == 1)
    {
      bool isConstFunc = (self.begin()->first.isConstTerm());
      if (isConstFunc) { return self.begin()->second.print(useParentheses); }
    }
    int count = -1;
    for (const auto& iter : self)
    {
      if (iter.second == PiPolynomial()) { continue; }
      ++count;
      auto coeff = iter.second;
      std::string plusString = " + ";
      if ((count != 0) && (coeff == PiRational(PiPolynomial(-1))))
      {
        plusString = " - "; coeff = -coeff;
      }
      if (count == 0) { plusString = ""; }
      strm << plusString;
      bool isConstantTerm = iter.first.isConstTerm();
      if (isConstantTerm || (coeff != PiRational(PiPolynomial(1))))
      {
        strm << coeff.print(useBrackets);   
      }
      if (iter.first.isConstTerm()) { continue; }
      if (coeff != PiRational(PiPolynomial(1)))
      {
        strm << " * ";
      }
      if (iter.first.xInd != 0)
      {
        strm << "x";
        if (iter.first.xInd != 1) { strm << "^" << iter.first.xInd; }
      }
      if (iter.first.yInd != 0)
      {
        strm << "y";
        if (iter.first.yInd != 1) { strm << "^" << iter.first.yInd; }
      }
      if (iter.first.zInd != 0)
      {
        strm << "z";
        if (iter.first.zInd != 1) { strm << "^" << iter.first.zInd; }
      }
      if (iter.first.ePiXInd != 0)
      {
        strm << "e^{Pi * ";
        if (iter.first.ePiXInd != 1) { strm << iter.first.ePiXInd.print(true) << " * "; }
        strm << "x}";
      }
      if (iter.first.trigPiXInd != TrigIndex())
      {
        strm << (iter.first.trigPiXInd.isCosine ? "cos" : "sin") << "(Pi * ";
        if (iter.first.trigPiXInd.self != 1) { strm << iter.first.trigPiXInd.self.print(true) << " * "; }
        strm << "x)";
      }
      if (iter.first.ePiYInd != 0)
      {
        strm << "e^{Pi * ";
        if (iter.first.ePiYInd != 1) { strm << iter.first.ePiYInd.print(true) << " * "; }
        strm << "y}";
      }
      if (iter.first.trigPiYInd != TrigIndex())
      {
        strm << (iter.first.trigPiYInd.isCosine ? "cos" : "sin") << "(Pi * ";
        if (iter.first.trigPiYInd.self != 1) { strm << iter.first.trigPiYInd.self.print(true) << " * "; }
        strm << "y)";
      }
      if (iter.first.ePiZInd != 0)
      {
        strm << "e^{Pi * ";
        if (iter.first.ePiZInd != 1) { strm << iter.first.ePiZInd.print(true) << " * "; }
        strm << "z}";
      }
      if (iter.first.trigPiZInd != TrigIndex())
      {
        strm << (iter.first.trigPiZInd.isCosine ? "cos" : "sin") << "(Pi * ";
        if (iter.first.trigPiZInd.self != 1) { strm << iter.first.trigPiZInd.self.print(true) << " * "; }
        strm << "z)";
      }
    }
    if (count < 0) { strm << "0"; }
    auto outStr = strm.str();
    trimParentheses(outStr, { '(', ')' });
    if (useParentheses) { outStr = std::string("(") + outStr + ")"; }
    return outStr;
  }

  AlgebraicPolynomial AlgebraicPolynomial::composeWith(const Matrix<ComplexQuadratic>& transform) const
  {
    AlgebraicPolynomial answer;

    if (transform.numRows() != 3) { throw std::invalid_argument("Transform must be 3x3 matrix."); return answer; }
    if (transform.numCols() != 3) { throw std::invalid_argument("Transform must be 3x3 matrix."); return answer; }

    const auto& AA = transform.at(0, 0);
    const auto& BB = transform.at(0, 1);
    const auto& CC = transform.at(0, 2);
    const auto& DD = transform.at(1, 0);
    const auto& EE = transform.at(1, 1);
    const auto& FF = transform.at(1, 2);
    const auto& GG = transform.at(2, 0);
    const auto& HH = transform.at(2, 1);
    const auto& II = transform.at(2, 2);

    PiRational zero = PiPolynomial(ComplexQuadratic(Rational(mp(0), mp(1))));
    PiRational one = PiPolynomial(ComplexQuadratic(Rational(mp(1), mp(1))));
    for (const auto& iter : self)
    {
      if (iter.second == PiRational()) { continue; }
      AlgebraicPolynomial term(iter.second);
      term = term * multinomial(one, PiPolynomial(AA), PiPolynomial(BB), PiPolynomial(CC), zero, iter.first.xInd);
      term = term * multinomial(one, PiPolynomial(DD), PiPolynomial(EE), PiPolynomial(FF), zero, iter.first.yInd);
      term = term * multinomial(one, PiPolynomial(GG), PiPolynomial(HH), PiPolynomial(II), zero, iter.first.zInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, AA * iter.first.ePiXInd, BB * iter.first.ePiXInd, CC * iter.first.ePiXInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, DD * iter.first.ePiYInd, EE * iter.first.ePiYInd, FF * iter.first.ePiYInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, GG * iter.first.ePiZInd, HH * iter.first.ePiZInd, II * iter.first.ePiZInd);
      if (iter.first.trigPiXInd.isCos())
      {
        term = term * cosPi_AX_plus_BY_plus_CZ(one, AA * iter.first.trigPiXInd.self, BB * iter.first.trigPiXInd.self, CC * iter.first.trigPiXInd.self);
      }
      else
      {
        term = term * sinPi_AX_plus_BY_plus_CZ(one, AA * iter.first.trigPiXInd.self, BB * iter.first.trigPiXInd.self, CC * iter.first.trigPiXInd.self);
      }
      if (iter.first.trigPiYInd.isCos())
      {
        term = term * cosPi_AX_plus_BY_plus_CZ(one, DD * iter.first.trigPiYInd.self, EE * iter.first.trigPiYInd.self, FF * iter.first.trigPiYInd.self);
      }
      else
      {
        term = term * sinPi_AX_plus_BY_plus_CZ(one, DD * iter.first.trigPiYInd.self, EE * iter.first.trigPiYInd.self, FF * iter.first.trigPiYInd.self);
      }
      if (iter.first.trigPiZInd.isCos())
      {
        term = term * cosPi_AX_plus_BY_plus_CZ(one, GG * iter.first.trigPiZInd.self, HH * iter.first.trigPiZInd.self, II * iter.first.trigPiZInd.self);
      }
      else
      {
        term = term * sinPi_AX_plus_BY_plus_CZ(one, GG * iter.first.trigPiZInd.self, HH * iter.first.trigPiZInd.self, II * iter.first.trigPiZInd.self);
      }
      answer = answer + term;
    }

    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::xToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.xInd = p;
    AlgebraicPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::yToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.yInd = p;
    AlgebraicPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::zToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.zInd = p;
    AlgebraicPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::multinomial(const PiRational& coeff,
      const PiRational& A, const PiRational& B, const PiRational& C, const PiRational& D, unsigned int p)
  {
    AlgebraicPolynomial answer;
    PiRational one = PiPolynomial(ComplexQuadratic(Rational(mp(1), mp(1))));
    for (unsigned int aa = 0; aa <= p; ++aa)
    {
      for (unsigned int bb = 0; bb <= (p - aa); ++bb)
      {
        for (unsigned int cc = 0; cc <= (p - aa - bb); ++cc)
        {
          PiRational termCoeff = A.pow(aa) * B.pow(bb) * C.pow(cc) * D.pow(p - aa - bb - cc);
          termCoeff = termCoeff * PiPolynomial(ComplexQuadratic(Rational(mp::binomialCoeff((int)p, (int)aa), mp(1))));
          termCoeff = termCoeff * PiPolynomial(ComplexQuadratic(Rational(mp::binomialCoeff((int)(p - aa), (int)bb), mp(1))));
          termCoeff = termCoeff * PiPolynomial(ComplexQuadratic(Rational(mp::binomialCoeff((int)(p - aa - bb), (int)cc), mp(1))));
          answer = answer + AlgebraicPolynomial::xToPower(termCoeff, aa) * AlgebraicPolynomial::yToPower(one, bb) * AlgebraicPolynomial::zToPower(one, cc);
        }
      }
    }
    return answer * coeff;
  }

  AlgebraicPolynomial AlgebraicPolynomial::eToTheATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiXInd = A.getRe();
    term.trigPiXInd = { A.getIm(), true };
    AlgebraicPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == BiquadraticNumber(0)) { return realTerm; }
    term.trigPiXInd = { A.getIm(), false };
    AlgebraicPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
  }

  AlgebraicPolynomial AlgebraicPolynomial::eToTheATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiYInd = A.getRe();
    term.trigPiYInd = { A.getIm(), true };
    AlgebraicPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == BiquadraticNumber(0)) { return realTerm; }
    term.trigPiYInd = { A.getIm(), false };
    AlgebraicPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
  }

  AlgebraicPolynomial AlgebraicPolynomial::eToTheATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiZInd = A.getRe();
    term.trigPiZInd = { A.getIm(), true };
    AlgebraicPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == BiquadraticNumber(0)) { return realTerm; }
    term.trigPiZInd = { A.getIm(), false };
    AlgebraicPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
  }

  AlgebraicPolynomial AlgebraicPolynomial::eToThePi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(ComplexQuadratic(1));
    return eToTheATimesPiX(coeff, A) * eToTheATimesPiY(one, B) * eToTheATimesPiZ(one, C);
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiX(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiX(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiY(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiY(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(-1, 4)));
    return eToTheATimesPiZ(coeffNew, A * ComplexQuadratic::sqrt(-1)) - eToTheATimesPiZ(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // sin(A)cos(B)cos(C) - sin(A)sin(B)sin(C)
    // + cos(A)sin(B)cos(C) + cos(A)cos(B)sin(C)
    return sinATimesPiX(coeff, A) * cosATimesPiY(one, B) * cosATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * sinATimesPiY(one, B) * sinATimesPiZ(one, C)
    + cosATimesPiX(coeff, A) * sinATimesPiY(one, B) * cosATimesPiZ(one, C)
    + cosATimesPiX(coeff, A) * cosATimesPiY(one, B) * sinATimesPiZ(one, C);
  }

  AlgebraicPolynomial AlgebraicPolynomial::cosATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiX(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiX(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::cosATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiY(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiY(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::cosATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiZ(coeffNew, A * ComplexQuadratic::sqrt(-1)) + eToTheATimesPiZ(coeffNew, -A * ComplexQuadratic::sqrt(-1));
  }

  AlgebraicPolynomial AlgebraicPolynomial::cosPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // cos(A)cos(B)cos(C) - cos(A)sin(B)sin(C)
    // - sin(A)sin(B)cos(C) - sin(A)cos(B)sin(C)
    return cosATimesPiX(coeff, A) * cosATimesPiY(one, B) * cosATimesPiZ(one, C)
    - cosATimesPiX(coeff, A) * sinATimesPiY(one, B) * sinATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * sinATimesPiY(one, B) * cosATimesPiZ(one, C)
    - sinATimesPiX(coeff, A) * cosATimesPiY(one, B) * sinATimesPiZ(one, C);
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinhATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiX(coeffNew, A) - eToTheATimesPiX(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinhATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiY(coeffNew, A) - eToTheATimesPiY(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinhATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = -coeff * PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 4)));
    return eToTheATimesPiZ(coeffNew, A) - eToTheATimesPiZ(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::sinhPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // sinh(A)cosh(B)cosh(C) + sinh(A)sinh(B)sinh(C)
    // + cosh(A)sinh(B)cosh(C) + cosh(A)cosh(B)sinh(C)
    return sinhATimesPiX(coeff, A) * coshATimesPiY(one, B) * coshATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * sinhATimesPiY(one, B) * sinhATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * sinhATimesPiY(one, B) * coshATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * coshATimesPiY(one, B) * sinhATimesPiZ(one, C);
  }

  AlgebraicPolynomial AlgebraicPolynomial::coshATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiX(coeffNew, A) + eToTheATimesPiX(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::coshATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiY(coeffNew, A) + eToTheATimesPiY(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::coshATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    auto coeffNew = coeff * PiPolynomial(ComplexQuadratic(Rational(1, 2)));
    return eToTheATimesPiZ(coeffNew, A) + eToTheATimesPiZ(coeffNew, -A);
  }

  AlgebraicPolynomial AlgebraicPolynomial::coshPi_AX_plus_BY_plus_CZ(const PiRational& coeff, const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C)
  {
    PiRational one(PiPolynomial(1), PiPolynomial(1));
    // cosh(A)cosh(B)cosh(C) + cosh(A)sinh(B)sinh(C)
    // + sinh(A)sinh(B)cosh(C) + sinh(A)cosh(B)sinh(C)
    return coshATimesPiX(coeff, A) * coshATimesPiY(one, B) * coshATimesPiZ(one, C)
    + coshATimesPiX(coeff, A) * sinhATimesPiY(one, B) * sinhATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * sinhATimesPiY(one, B) * coshATimesPiZ(one, C)
    + sinhATimesPiX(coeff, A) * coshATimesPiY(one, B) * sinhATimesPiZ(one, C);
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator+() const
  {
    return *this;
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.self)
    {
      iter.second = -iter.second;
    }
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator+(const AlgebraicPolynomial& rhs) const
  {
    AlgebraicPolynomial answer = *this;

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

  AlgebraicPolynomial AlgebraicPolynomial::operator-(const AlgebraicPolynomial& rhs) const
  {
    return (*this) + (-rhs);
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator*(const AlgebraicPolynomial& rhs) const
  {
    AlgebraicPolynomial answer;

    for (const auto& iter : self)
    {
      for (const auto& jter : rhs.self)
      {
        auto summand = iter.second * jter.second;
        auto monomials = iter.first.trigSum(jter.first);
        for (const auto& kter : monomials)
        {
          auto currentSummand = summand * PiPolynomial(kter.second);
          auto kk = kter.first;
          if (answer.self.find(kk) == answer.self.end())
          { 
            answer.self[kk] = currentSummand;
            continue;
          }
          answer.self[kk] = answer.self[kk] + currentSummand;
        }
      }
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator*(const PiPolynomial& rhs) const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self) { answer.self[iter.first] = iter.second * PiRational(PiPolynomial(rhs)); }
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    AlgebraicPolynomial answer;
    Monomial constTerm;
    answer.self[constTerm] = PiPolynomial(ComplexQuadratic(1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool AlgebraicPolynomial::operator==(const AlgebraicPolynomial& rhs) const
  {
    auto diff = (*this) - rhs;
    for (const auto& iter : diff.self)
    {
      if (iter.second != PiPolynomial(0)) { return false; }
    }
    return true;
  }

  bool AlgebraicPolynomial::operator!=(const AlgebraicPolynomial& rhs) const
  {
    return !((*this) == rhs);
  }


  AlgebraicPolynomial AlgebraicPolynomial::partial_x() const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.xInd));
      if (newIndex.first.xInd > 0) { --(newIndex.first.xInd); }
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      if (iter.first.ePiXInd != 0)
      {
        newIndex = iter;
        newIndex.second = newIndex.second * (PiPolynomial(newIndex.first.ePiXInd) * PiPolynomial(1, 1));
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }

      if (iter.first.trigPiXInd != TrigIndex())
      {
        newIndex = iter;
        newIndex.first.trigPiXInd.isCosine = !iter.first.trigPiXInd.isCosine;
        newIndex.second = newIndex.second * (PiPolynomial(iter.first.trigPiXInd.self) * PiPolynomial(1, 1));
        if (iter.first.trigPiXInd.isCosine) { newIndex.second = -newIndex.second; }
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_y() const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.yInd));
      if (newIndex.first.yInd > 0) { --(newIndex.first.yInd); }
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      if (iter.first.ePiYInd != 0)
      {
        newIndex = iter;
        newIndex.second = newIndex.second * (PiPolynomial(newIndex.first.ePiYInd) * PiPolynomial(1, 1));
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }

      if (iter.first.trigPiYInd != TrigIndex())
      {
        newIndex = iter;
        newIndex.first.trigPiYInd.isCosine = !iter.first.trigPiYInd.isCosine;
        newIndex.second = newIndex.second * (PiPolynomial(iter.first.trigPiYInd.self) * PiPolynomial(1, 1));
        if (iter.first.trigPiYInd.isCosine) { newIndex.second = -newIndex.second; }
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_z() const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(iter.first.zInd));
      if (newIndex.first.zInd > 0) { --(newIndex.first.zInd); }
      if (answer.self.find(newIndex.first) != answer.self.end())
      {
        answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }

      if (iter.first.ePiZInd != 0)
      {
        newIndex = iter;
        newIndex.second = newIndex.second * (PiPolynomial(newIndex.first.ePiZInd) * PiPolynomial(1, 1));
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }

      if (iter.first.trigPiZInd != TrigIndex())
      {
        newIndex = iter;
        newIndex.first.trigPiZInd.isCosine = !iter.first.trigPiZInd.isCosine;
        newIndex.second = newIndex.second * (PiPolynomial(iter.first.trigPiZInd.self) * PiPolynomial(1, 1));
        if (iter.first.trigPiZInd.isCosine) { newIndex.second = -newIndex.second; }
        if (answer.self.find(newIndex.first) != answer.self.end())
        {
          answer.self[newIndex.first] = answer.self[newIndex.first] + newIndex.second;
        }
        else { answer.self[newIndex.first] = newIndex.second; }
      }
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::laplacian() const
  {
    auto xPortion = (*this).partial_x().partial_x();
    auto yPortion = (*this).partial_y().partial_y();
    auto zPortion = (*this).partial_z().partial_z();
    return xPortion + yPortion + zPortion;
  }

  bool AlgebraicPolynomial::isLaplaceEigenfunction(PiRational& eigenvalue) const
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

  bool AlgebraicPolynomial::isHarmonic() const
  {
    return (laplacian() == AlgebraicPolynomial(PiPolynomial(0)));
  }

  bool AlgebraicPolynomial::tryEvaluateAtX(const ComplexQuadratic& xVal, AlgebraicPolynomial& output) const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      auto newKey = iter.first;
      auto newVal = iter.second;
      if (newKey.xInd != 0)
      {
        newVal = newVal * PiPolynomial(xVal.pow(newKey.xInd));
        newKey.xInd = 0;
      }
      if (newKey.ePiXInd != 0)
      {
        if (xVal != 0) { return false; }
        newKey.ePiXInd = BiquadraticNumber();
      }
      if (newKey.trigPiXInd != TrigIndex())
      {
        ComplexQuadratic trigInput = xVal * newKey.trigPiXInd.self;
        if (trigInput.getIm() != 0) { return false; }
        Rational rationalVal;
        bool isRational = trigInput.getRe().getRational(rationalVal);
        if (!isRational) { return false; }
        if (newKey.trigPiXInd.isCos())
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetCosine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        else
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetSine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        newKey.trigPiXInd = TrigIndex();
      }
      auto newIter = answer.self.find(newKey);
      if (newIter == answer.self.end()) { answer.self[newKey] = newVal; }
      else { newIter->second = newIter->second + newVal; }
    }
    answer.clean();
    output = answer;
    return true;
  }

  bool AlgebraicPolynomial::tryEvaluateAtY(const ComplexQuadratic& yVal, AlgebraicPolynomial& output) const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      auto newKey = iter.first;
      auto newVal = iter.second;
      if (newKey.yInd != 0)
      {
        newVal = newVal * PiPolynomial(yVal.pow(newKey.yInd));
        newKey.yInd = 0;
      }
      if (newKey.ePiYInd != 0)
      {
        if (yVal != 0) { return false; }
        newKey.ePiYInd = BiquadraticNumber();
      }
      if (newKey.trigPiYInd != TrigIndex())
      {
        ComplexQuadratic trigInput = yVal * newKey.trigPiYInd.self;
        if (trigInput.getIm() != 0) { return false; }
        Rational rationalVal;
        bool isRational = trigInput.getRe().getRational(rationalVal);
        if (!isRational) { return false; }
        if (newKey.trigPiYInd.isCos())
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetCosine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        else
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetSine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        newKey.trigPiYInd = TrigIndex();
      }
      auto newIter = answer.self.find(newKey);
      if (newIter == answer.self.end()) { answer.self[newKey] = newVal; }
      else { newIter->second = newIter->second + newVal; }
    }
    answer.clean();
    output = answer;
    return true;
  }

  bool AlgebraicPolynomial::tryEvaluateAtZ(const ComplexQuadratic& zVal, AlgebraicPolynomial& output) const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      auto newKey = iter.first;
      auto newVal = iter.second;
      if (newKey.zInd != 0)
      {
        newVal = newVal * PiPolynomial(zVal.pow(newKey.zInd));
        newKey.zInd = 0;
      }
      if (newKey.ePiZInd != 0)
      {
        if (zVal != 0) { return false; }
        newKey.ePiZInd = BiquadraticNumber();
      }
      if (newKey.trigPiZInd != TrigIndex())
      {
        ComplexQuadratic trigInput = zVal * newKey.trigPiZInd.self;
        if (trigInput.getIm() != 0) { return false; }
        Rational rationalVal;
        bool isRational = trigInput.getRe().getRational(rationalVal);
        if (!isRational) { return false; }
        if (newKey.trigPiZInd.isCos())
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetCosine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        else
        {
          BiquadraticNumber factor;
          bool trigSuccess = BiquadraticNumber::tryGetSine(rationalVal, factor);
          if (!trigSuccess) { return false; }
          newVal = newVal * PiPolynomial(ComplexQuadratic(factor));
        }
        newKey.trigPiZInd = TrigIndex();
      }
      auto newIter = answer.self.find(newKey);
      if (newIter == answer.self.end()) { answer.self[newKey] = newVal; }
      else { newIter->second = newIter->second + newVal; }
    }
    answer.clean();
    output = answer;
    return true;
  }

  bool AlgebraicPolynomial::tryEvaluateAtXYZ(const ComplexQuadratic& xVal, const ComplexQuadratic& yVal, const ComplexQuadratic& zVal, PiRational& output) const
  {
    AlgebraicPolynomial answerX, answerY, answerZ;
    bool success = tryEvaluateAtX(xVal, answerX);
    if (!success) { return false; }
    success = answerX.tryEvaluateAtY(yVal, answerY);
    if (!success) { return false; }
    success = answerY.tryEvaluateAtZ(zVal, answerZ);
    if (!success) { return false; }
    if (answerZ.self.size() > 1) { return false; }
    if (answerZ.self.size() < 1) { output = PiRational(); return true; }
    if (answerZ.self.find(Monomial()) == answerZ.self.end()) { return false; }
    output = answerZ.self.at(Monomial());
    return true;
  }
}
