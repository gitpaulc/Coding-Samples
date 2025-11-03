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

  AlgebraicPolynomial::Monomial AlgebraicPolynomial::Monomial::operator*(const AlgebraicPolynomial::Monomial& rhs) const
  {
    Monomial answer = *this;
    for (const auto& it : rhs.indices)
    {
      auto jt = answer.indices.find(it.first);
      if (jt == answer.indices.end())
      {
        answer.indices[it.first] = it.second;
        continue;
      }
      jt->second = jt->second + it.second;
    }
    return answer;
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

    for (const auto& it : self)
    {
      if (it.second == PiRational()) { continue; }
      auto coeff = it.second;
      auto monomial = it.first;
      monomial.clean();
      auto jt = answer.self.find(monomial);
      if (jt == answer.self.end())
      {
        answer.self[monomial] = coeff;
        continue;
      }
      jt->second = jt->second + coeff;
    }

    *this = answer;
  }

  AlgebraicPolynomial::AlgebraicPolynomial(const PiRational& coeff)
  {
    if (coeff != PiRational())
    {
      Monomial constTerm;
      self[constTerm] = coeff;
    }
  }

  unsigned int AlgebraicPolynomial::getDimension() const
  {
    int maxDimension = 0;
    for (const auto& it : self)
    {
      int currentDim = it.first.getDimension();
      if (currentDim > maxDimension) { maxDimension = currentDim; }
    }
    return maxDimension;
  }

  bool AlgebraicPolynomial::isConstant(PiRational* evaluatedValue) const
  {
    bool answer = true;
    PiRational theVal;
    for (const auto& it : self)
    {
      if (it.second == PiRational()) { continue; }
      if (!(it.first.isConstTerm())) { return false; }
      theVal = it.second;
    }
    if (answer && (evaluatedValue != nullptr)) { *evaluatedValue = theVal; }
    return answer;
  }

  std::string AlgebraicPolynomial::print(bool useParentheses, bool detectLowDimension) const
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
      bool lowDimension = detectLowDimension && (getDimension() <= 4);
      std::vector<std::string> vars = { "x", "y", "z", "w" };
      for (const auto& jt : iter.first.indices)
      {
        if (lowDimension && (jt.first < vars.size()))
        {
          strm << vars[jt.first];
          if (jt.second != 1) { strm << "^" << jt.second; }
          continue;
        }
        if (jt.second != 1) { strm << "("; }
        strm << "x_" << jt.first;
        if (jt.second != 1) { strm << ")^" << jt.second; }
      }
    }
    if (count < 0) { strm << "0"; }
    auto outStr = strm.str();
    trimParentheses(outStr, { '(', ')' });
    if (useParentheses) { outStr = std::string("(") + outStr + ")"; }
    return outStr;
  }

  AlgebraicPolynomial AlgebraicPolynomial::x_iToPower(const PiRational& coeff, unsigned int i, unsigned int p)
  {
    Monomial term;
    term.indices[i] = p;
    AlgebraicPolynomial answer;
    answer.self[term] = coeff;
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::xToPower(const PiRational& coeff, unsigned int p)
  {
    return x_iToPower(coeff, 0, p);
  }

  AlgebraicPolynomial AlgebraicPolynomial::yToPower(const PiRational& coeff, unsigned int p)
  {
    return x_iToPower(coeff, 1, p);
  }

  AlgebraicPolynomial AlgebraicPolynomial::zToPower(const PiRational& coeff, unsigned int p)
  {
    return x_iToPower(coeff, 2, p);
  }

  AlgebraicPolynomial AlgebraicPolynomial::wToPower(const PiRational& coeff, unsigned int p)
  {
    return x_iToPower(coeff, 3, p);
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
      auto jt = answer.self.find(iter.first);
      if (jt == answer.self.end())
      {
        answer.self[iter.first] = iter.second;
        continue;
      }
      jt->second = jt->second + iter.second;
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
        auto coeff = iter.second * jter.second;
        auto summand = iter.first * jter.first;
        auto kter = answer.self.find(summand);
        if (kter == answer.self.end())
        {
          answer.self[summand] = coeff;
          continue;
        }
        kter->second = kter->second + coeff;
      }
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::operator*(const PiPolynomial& rhs) const
  {
    AlgebraicPolynomial answer;
    if (rhs == PiPolynomial()) { return answer; }
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


  AlgebraicPolynomial AlgebraicPolynomial::partial_deriv(unsigned int index) const
  {
    AlgebraicPolynomial answer;
    for (const auto& iter : self)
    {
      std::pair<Monomial, PiRational> newIndex = iter;
      unsigned int power = 0;
      {
        auto jt = iter.first.indices.find(index);
        if (jt != iter.first.indices.end())
        {
          power = jt->second;
        }
      }
      newIndex.second = newIndex.second * PiPolynomial(ComplexQuadratic(power));
      if (power > 0)
      {
        auto jt = newIndex.first.indices.find(index);
        if (jt != newIndex.first.indices.end())
        {
          --(jt->second);
        }
      }
      auto jt = answer.self.find(newIndex.first);
      if (jt != answer.self.end())
      {
        jt->second = jt->second + newIndex.second;
      }
      else { answer.self[newIndex.first] = newIndex.second; }
      
    }
    answer.clean();
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_x() const
  {
    return partial_deriv(0);
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_y() const
  {
    return partial_deriv(1);
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_z() const
  {
    return partial_deriv(2);
  }

  AlgebraicPolynomial AlgebraicPolynomial::partial_w() const
  {
    return partial_deriv(3);
  }

  AlgebraicPolynomial AlgebraicPolynomial::laplacian() const
  {
    AlgebraicPolynomial answer;
    int dim = (int)getDimension();
    for (int ii = 0; ii < dim - 1; ++ii)
    {
      answer = answer + (*this).partial_deriv(ii).partial_deriv(ii);
    }
    return answer;
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

  AlgebraicPolynomial AlgebraicPolynomial::evaluateAt(const std::vector<PiRational>& input) const
  {
    AlgebraicPolynomial answer;
    if (input.size() == 0) { throw std::logic_error("Input must contain at least one variable."); }
    if (input.size() == 1)
    {
      std::map<unsigned int, PiRational> singleInput;
      singleInput[0] = input[0];
      return evaluateAt(singleInput);
    }
    auto singleInput = input[input.size() - 1];
    auto truncated = input;
    truncated.resize(input.size() - 1);
    answer = evaluateAt(truncated);
    answer = answer.evaluateAt({ singleInput });
    return answer;
  }

  AlgebraicPolynomial AlgebraicPolynomial::evaluateAt(const std::map<unsigned int, PiRational>& input) const
  {
    AlgebraicPolynomial answer;
    if (input.size() == 0) { throw std::logic_error("Input must contain at least one variable."); }
    if (input.size() == 1)
    {
      auto varIndex = input.begin()->first;
      auto varValue = input.begin()->second;
      for (const auto& iter : self)
      {
        auto& indexes = iter.first.indices;
        Monomial monomial;
        auto coeff = iter.second;
        for (const auto& jter : indexes)
        {
          if (jter.first == varIndex)
          {
            coeff = coeff * varValue.pow(jter.second);
            continue;
          }
          monomial.indices[jter.first] = jter.second;
        }
        AlgebraicPolynomial summand;
        summand.self[monomial] = coeff;
        answer = answer + summand;
      }
      return answer;
    }
    std::map<unsigned int, PiRational> singleInput, truncated;
    {
      int ii = -1;
      for (const auto& iter : input)
      {
        ++ii;
        if (ii == 0)
        {
          singleInput[iter.first] = iter.second;
          continue;
        }
        truncated[iter.first] = iter.second;
      }
    }
    answer = evaluateAt(truncated);
    answer = answer.evaluateAt({ singleInput });
    return answer;
  }

  bool AlgebraicPolynomial::tryEvaluate(const std::vector<PiRational>& input, PiRational& output) const
  {
    AlgebraicPolynomial answer = evaluateAt(input);
    PiRational constAns;
    bool itIsConst = answer.isConstant(&constAns);
    if (!itIsConst) { return false; }
    output = constAns;
    return true;
  }
}
