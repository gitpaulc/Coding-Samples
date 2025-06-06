/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "fn_polynomial.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  bool FnPolynomial::TrigIndex::isCos() const
  {
    if (self == QuadraticNumber(0)) { return true; }
    return isCosine;
  }

  bool FnPolynomial::TrigIndex::operator==(const FnPolynomial::TrigIndex& rhs) const
  {
    if (self == QuadraticNumber(0)) { return (self == rhs.self); }
    if (rhs.self == QuadraticNumber(0)) { return (self == rhs.self); }
    if (self != rhs.self) { return false; }
    return (isCosine == rhs.isCosine);
  }

  bool FnPolynomial::TrigIndex::operator!=(const FnPolynomial::TrigIndex& rhs) const
  {
    return !((*this) == rhs);
  }

  bool FnPolynomial::TrigIndex::operator<(const FnPolynomial::TrigIndex& rhs) const
  {
    if (self < rhs.self) { return true; }
    if (rhs.self < self) { return false; }
    if (self == QuadraticNumber(0)) { return false; } // They are both treated as cosine.
    if (isCosine == rhs.isCosine) { return false; }
    // One must be true, one must be false. false < true:
    return (isCosine == false);
  }

  bool FnPolynomial::TrigIndex::operator>(const FnPolynomial::TrigIndex& rhs) const { return (rhs < (*this)); }

  bool FnPolynomial::Monomial::isConstTerm() const
  {
    if (xInd != 0) { return false; }
    if (yInd != 0) { return false; }
    if (zInd != 0) { return false; }
    if (ePiXInd != 0) { return false; }
    if (ePiYInd != 0) { return false; }
    if (ePiZInd != 0) { return false; }
    if (trigPiXInd != TrigIndex()) { return false; }
    if (trigPiYInd != TrigIndex()) { return false; }
    if (trigPiZInd != TrigIndex()) { return false; }
    return true;
  }

  std::map<FnPolynomial::Monomial, QuadraticNumber> FnPolynomial::Monomial::trigSum(const Monomial& rhs) const
  {
    std::map<Monomial, QuadraticNumber> answers;
    for (int i = 0; i < 8; ++i)
    {
      Monomial answer;
      answer.xInd = xInd + rhs.xInd;
      answer.yInd = yInd + rhs.yInd;
      answer.zInd = zInd + rhs.zInd;
      answer.ePiXInd = ePiXInd + rhs.ePiXInd;
      answer.ePiYInd = ePiYInd + rhs.ePiYInd;
      answer.ePiZInd = ePiZInd + rhs.ePiZInd;
      bool xRight = (((i / 4) % 2) == 1) ? true : false;
      bool yRight = (((i / 2) % 2) == 1) ? true : false;
      bool zRight = ((i % 2) == 1) ? true : false;
      /*
      cos(ax)cos(Ax) = (1/2)cos((a + A)x) + (1/2)cos((a - A)x)
      cos(ax)sin(Ax) = (1/2)sin((a + A)x) - (1/2)sin((a - A)x)
      sin(ax)cos(Ax) = (1/2)sin((a + A)x) + (1/2)sin((a - A)x)
      sin(ax)sin(Ax) = (1/2)cos((a - A)x) - (1/2)cos((a + A)x)
      */
      QuadraticNumber coeff(Rational(1, 1));
      if (trigPiXInd.isCos() == rhs.trigPiXInd.isCos())
      {
        answer.trigPiXInd.isCosine = true;
        if (trigPiXInd.isCos()) // cos(ax)cos(Ax)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiXInd.self = xRight ? (trigPiXInd.self - rhs.trigPiXInd.self) : (trigPiXInd.self + rhs.trigPiXInd.self);
        }
        else // sin(ax)sin(Ax)
        {
          coeff = coeff * Rational(1, 2); if (xRight) { coeff = -coeff; }
          answer.trigPiXInd.self = xRight ? (trigPiXInd.self + rhs.trigPiXInd.self) : (trigPiXInd.self - rhs.trigPiXInd.self);
        }
      }
      else
      {
        answer.trigPiXInd.isCosine = false;
        if (trigPiXInd.isCos()) // cos(ax)sin(Ax)
        {
          coeff = coeff * Rational(1, 2); if (xRight) { coeff = -coeff; }
          answer.trigPiXInd.self = xRight ? (trigPiXInd.self - rhs.trigPiXInd.self) : (trigPiXInd.self + rhs.trigPiXInd.self);
          if (answer.trigPiXInd.self == QuadraticNumber(0)) { continue; }
        }
        else // sin(ax)cos(Ax)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiXInd.self = xRight ? (trigPiXInd.self - rhs.trigPiXInd.self) : (trigPiXInd.self + rhs.trigPiXInd.self);
          if (answer.trigPiXInd.self == QuadraticNumber(0)) { continue; }
        }
      }
      if (trigPiYInd.isCos() == rhs.trigPiYInd.isCos())
      {
        answer.trigPiYInd.isCosine = true;
        if (trigPiYInd.isCos()) // cos(ay)cos(Ay)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiYInd.self = yRight ? (trigPiYInd.self - rhs.trigPiYInd.self) : (trigPiYInd.self + rhs.trigPiYInd.self);
        }
        else // sin(ay)sin(Ay)
        {
          coeff = coeff * Rational(1, 2); if (yRight) { coeff = -coeff; }
          answer.trigPiYInd.self = yRight ? (trigPiYInd.self + rhs.trigPiYInd.self) : (trigPiYInd.self - rhs.trigPiYInd.self);
        }
      }
      else
      {
        answer.trigPiYInd.isCosine = false;
        if (trigPiYInd.isCos()) // cos(ay)sin(Ay)
        {
          coeff = coeff * Rational(1, 2); if (yRight) { coeff = -coeff; }
          answer.trigPiYInd.self = yRight ? (trigPiYInd.self - rhs.trigPiYInd.self) : (trigPiYInd.self + rhs.trigPiYInd.self);
          if (answer.trigPiYInd.self == QuadraticNumber(0)) { continue; }
        }
        else // sin(ay)cos(Ay)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiYInd.self = yRight ? (trigPiYInd.self - rhs.trigPiYInd.self) : (trigPiYInd.self + rhs.trigPiYInd.self);
          if (answer.trigPiYInd.self == QuadraticNumber(0)) { continue; }
        }
      }
      if (trigPiZInd.isCos() == rhs.trigPiZInd.isCos())
      {
        answer.trigPiZInd.isCosine = true;
        if (trigPiZInd.isCos()) // cos(az)cos(Az)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiZInd.self = zRight ? (trigPiZInd.self - rhs.trigPiZInd.self) : (trigPiZInd.self + rhs.trigPiZInd.self);
        }
        else // sin(az)sin(Az)
        {
          coeff = coeff * Rational(1, 2); if (zRight) { coeff = -coeff; }
          answer.trigPiZInd.self = zRight ? (trigPiZInd.self + rhs.trigPiZInd.self) : (trigPiZInd.self - rhs.trigPiZInd.self);
        }
      }
      else
      {
        answer.trigPiZInd.isCosine = false;
        if (trigPiZInd.isCos()) // cos(az)sin(Az)
        {
          coeff = coeff * Rational(1, 2); if (zRight) { coeff = -coeff; }
          answer.trigPiZInd.self = zRight ? (trigPiZInd.self - rhs.trigPiZInd.self) : (trigPiZInd.self + rhs.trigPiZInd.self);
          if (answer.trigPiZInd.self == QuadraticNumber(0)) { continue; }
        }
        else // sin(az)cos(Az)
        {
          coeff = coeff * Rational(1, 2);
          answer.trigPiZInd.self = zRight ? (trigPiZInd.self - rhs.trigPiZInd.self) : (trigPiZInd.self + rhs.trigPiZInd.self);
          if (answer.trigPiZInd.self == QuadraticNumber(0)) { continue; }
        }
      }
      answers[answer] = coeff;
    }
    return answers;
  }

  bool FnPolynomial::Monomial::operator<(const FnPolynomial::Monomial& rhs) const
  {
    if (zInd < rhs.zInd) { return true; }
    if (zInd > rhs.zInd) { return false; }
    if (ePiZInd < rhs.ePiZInd) { return true; }
    if (ePiZInd > rhs.ePiZInd) { return false; }
    if (trigPiZInd < rhs.trigPiZInd) { return true; }
    if (trigPiZInd > rhs.trigPiZInd) { return false; }
    if (yInd < rhs.yInd) { return true; }
    if (yInd > rhs.yInd) { return false; }
    if (ePiYInd < rhs.ePiYInd) { return true; }
    if (ePiYInd > rhs.ePiYInd) { return false; }
    if (trigPiYInd < rhs.trigPiYInd) { return true; }
    if (trigPiYInd > rhs.trigPiYInd) { return false; }
    if (xInd < rhs.xInd) { return true; }
    if (xInd > rhs.xInd) { return false; }
    if (ePiXInd < rhs.ePiXInd) { return true; }
    if (ePiXInd > rhs.ePiXInd) { return false; }
    if (trigPiXInd < rhs.trigPiXInd) { return true; }
    if (trigPiXInd > rhs.trigPiXInd) { return false; }
    return false; // They are equal.
  }

  std::map<FnPolynomial::Monomial, PiRational>::iterator FnPolynomial::trigFind(const FnPolynomial::Monomial& ind,
    bool& xNegative, bool& yNegative, bool& zNegative)
  {
    xNegative = false;
    yNegative = false;
    zNegative = false;
    for (int i = 0; i < 8; ++i)
    {
      bool xNeg = (((i / 4) % 2) == 1) ? true : false;
      bool yNeg = (((i / 2) % 2) == 1) ? true : false;
      bool zNeg = ((i % 2) == 1) ? true : false;
      auto indB = ind;
      if (xNeg) { indB.trigPiXInd.self = -indB.trigPiXInd.self; xNegative = true; }
      if (yNeg) { indB.trigPiYInd.self = -indB.trigPiYInd.self; yNegative = true; }
      if (zNeg) { indB.trigPiZInd.self = -indB.trigPiZInd.self; zNegative = true; }
      auto iter = self.find(indB);
      if (iter != self.end()) { return iter; }
    }
    return self.end();
  }

  void FnPolynomial::clean()
  {
    FnPolynomial answer;
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
      if (iter.first.ePiXInd != 0) { strm << "e^{Pi * " << iter.first.ePiXInd.print(true) << " * x}"; }
      if (iter.first.trigPiXInd != TrigIndex())
      {
        strm << (iter.first.trigPiXInd.isCosine ? "cos" : "sin") << "(" << iter.first.trigPiXInd.self.print(true) << " * x)";
      }
      if (iter.first.ePiYInd != 0) { strm << "e^{Pi * " << iter.first.ePiYInd.print(true) << " * y}"; }
      if (iter.first.trigPiYInd != TrigIndex())
      {
        strm << (iter.first.trigPiYInd.isCosine ? "cos" : "sin") << "(" << iter.first.trigPiYInd.self.print(true) << " * y)";
      }
      if (iter.first.ePiZInd != 0) { strm << "e^{Pi * " << iter.first.ePiZInd.print(true) << " * z}"; }
      if (iter.first.trigPiZInd != TrigIndex())
      {
        strm << (iter.first.trigPiZInd.isCosine ? "cos" : "sin") << "(" << iter.first.trigPiZInd.self.print(true) << " * z)";
      }
    }
    if (count < 0) { strm << "0"; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  FnPolynomial FnPolynomial::composeWith(const Matrix<ComplexQuadratic>& transform) const
  {
    FnPolynomial answer;

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
      FnPolynomial term(iter.second);
      term = term * multinomial(one, PiPolynomial(AA), PiPolynomial(BB), PiPolynomial(CC), zero, iter.first.xInd);
      term = term * multinomial(one, PiPolynomial(DD), PiPolynomial(EE), PiPolynomial(FF), zero, iter.first.yInd);
      term = term * multinomial(one, PiPolynomial(GG), PiPolynomial(HH), PiPolynomial(II), zero, iter.first.zInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, AA * iter.first.ePiXInd, BB * iter.first.ePiXInd, CC * iter.first.ePiXInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, DD * iter.first.ePiYInd, EE * iter.first.ePiYInd, FF * iter.first.ePiYInd);
      term = term * eToThePi_AX_plus_BY_plus_CZ(one, GG * iter.first.ePiZInd, HH * iter.first.ePiZInd, II * iter.first.ePiZInd);
      answer = answer + term;
    }

    answer.clean();
    return answer;
  }

  FnPolynomial FnPolynomial::xToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.xInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::yToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.yInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::zToPower(const PiRational& coeff, unsigned int p)
  {
    Monomial term;
    term.zInd = p;
    FnPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  FnPolynomial FnPolynomial::multinomial(const PiRational& coeff,
      const PiRational& A, const PiRational& B, const PiRational& C, const PiRational& D, unsigned int p)
  {
    FnPolynomial answer;
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
          answer = answer + FnPolynomial::xToPower(termCoeff, aa) * FnPolynomial::yToPower(one, bb) * FnPolynomial::zToPower(one, cc);
        }
      }
    }
    return answer * coeff;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiX(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiXInd = A.getRe();
    term.trigPiXInd = { A.getIm(), true };
    FnPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == QuadraticNumber(0)) { return realTerm; }
    term.trigPiXInd = { A.getIm(), false };
    FnPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiY(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiYInd = A.getRe();
    term.trigPiYInd = { A.getIm(), true };
    FnPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == QuadraticNumber(0)) { return realTerm; }
    term.trigPiYInd = { A.getIm(), false };
    FnPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
  }

  FnPolynomial FnPolynomial::eToTheATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A)
  {
    Monomial term;
    term.ePiZInd = A.getRe();
    term.trigPiZInd = { A.getIm(), true };
    FnPolynomial realTerm;
    realTerm.self[term] = coeff;
    if (A.getIm() == QuadraticNumber(0)) { return realTerm; }
    term.trigPiZInd = { A.getIm(), false };
    FnPolynomial imTerm;
    imTerm.self[term] = coeff * PiPolynomial(ComplexQuadratic::sqrt(-1));
    return realTerm + imTerm;
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
        auto monomials = iter.first.trigSum(jter.first);
        for (const auto& kter : monomials)
        {
          auto currentSummand = summand * PiPolynomial(kter.second);
          auto kk = iter.first;
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

  FnPolynomial FnPolynomial::partial_y() const
  {
    FnPolynomial answer;
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

  FnPolynomial FnPolynomial::partial_z() const
  {
    FnPolynomial answer;
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
