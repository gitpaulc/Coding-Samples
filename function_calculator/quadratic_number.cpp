/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "quadratic_number.h"

#include "complex_quadratic.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  QuadraticNumber::QuadraticNumber(const Rational& number)
  {
    if (number != 0)
    {
      *this = QuadraticNumber::sqrt(1);
      content[1] = content[1] * number;
    }
  }

  std::pair<double, double> QuadraticNumber::get() const
  {
    double answer = 0.0;
    for (const auto& iter : content)
    {
      double val = iter.second.get().first;
      if (val == 0) { continue; }
      double radicand = iter.first.toInt();
      if (iter.first < 0) { throw std::invalid_argument("\nRadicands should be nonnegative."); radicand = -radicand; }
      answer += val * std::sqrt(radicand);
    }
    return { answer, 0.0 };
  }

  std::pair<QuadraticNumber, mp> QuadraticNumber::factorAsIntegral() const
  {
    std::pair<QuadraticNumber, mp> answer;
    answer.first = *this;
    answer.second = mp(1);
    for (const auto& iter : answer.first.content)
    {
      answer.second = answer.second * iter.second.denominator();
    }
    for (auto& iter : answer.first.content)
    {
      iter.second = iter.second * Rational(answer.second, mp(1));
    }
    return answer;
  }

  bool QuadraticNumber::getRational(Rational& self) const
  {
    if (content.size() == 0) { self = Rational(0, 1); return true; }
    if (content.size() > 1) { return false; }
    if (content.find(1) == content.end()) { return false; }
    self = content.at(1);
    return true;
  }

  Matrix<Rational> QuadraticNumber::getMultiplicationMatrix(std::map<mp, int>& root2Index, std::map<int, mp>& index2Root) const
  {
    root2Index = std::map<mp, int>();
    index2Root = std::map<int, mp>();
    {
      std::set<mp> rootsSoFar;
      const int NN = (int)content.size();
      auto current = *this;
      for (int II = 0; II < (NN + 1); ++II)
      {
        for (const auto& iter : current.content)
        {
          rootsSoFar.insert(iter.first);
        }
        current = current * (*this);
      }
      int ii = 0;
      for (const auto& rootSoFar : rootsSoFar)
      {
        root2Index[rootSoFar] = ii; index2Root[ii] = rootSoFar; ++ii;
      }
    }
    const int dimMatrix = (int)root2Index.size();
    auto answer = Matrix<Rational>::zeroMatrix(dimMatrix);
    for (const auto& iter : content)
    {
      Matrix<Rational> summand;
      const auto& radA = iter.first;
      for (int ii = 0; ii < dimMatrix; ++ii)
      {
        std::vector<Rational> row(dimMatrix, 0);
        const auto& radB = index2Root[ii];
        auto root = radA * radB;
        auto sqrtSplit = root.separateSquaredPart();
        row[root2Index[sqrtSplit.second]] = Rational(sqrtSplit.first, 1);
        summand.addRow(row);
      }
      answer = answer + summand.transpose() * iter.second;
    }
    return answer; //
  }

  std::string QuadraticNumber::print(bool useParentheses) const
  {
    std::stringstream strm;
    int count = -1;
    if (useParentheses) { strm << "("; }
    if (content.size() == 0) { strm << "0"; }
    for (const auto& iter : content)
    {
      auto val = iter.second;
      if (val == 0) { continue; }
      ++count;
      if (count > 0)
      {
        if (val >= 0) { strm << " + "; }
        else
        {
          val = -val;
          strm << " - ";
        }
      }
      bool coeffIsOne = (val == 1);
      auto radicand = iter.first;
      bool printCoeffParents = (val.denominator() != 1) && (radicand != 1);
      if ((radicand == 1) || (!coeffIsOne)) { strm << val.print(printCoeffParents); }
      if (radicand == 1) { continue; }
      bool complex = false;
      if (radicand < 0) { radicand = -radicand; complex = true; }
      if (!coeffIsOne) { strm << " * "; }
      if (!complex || (radicand != 1))
      {
        strm << "Sqrt(" << radicand << ")";
        if (complex) { strm << " * "; }
      }
      if (complex) { strm << "i"; }
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  QuadraticNumber QuadraticNumber::sqrt(const Rational& radicand)
  {
    QuadraticNumber answer;
    if (radicand == Rational(0, 1)) { return answer; }
    if (radicand < 0) { throw std::invalid_argument("Radicand should be nonnegative."); return answer; }
    Rational coefficient(1, radicand.denominator());
    mp key = radicand.numerator() * radicand.denominator();
    auto primes = key.primeFactorization();
    for (const auto& iter : primes)
    {
      auto& factor = iter.first;
      if (factor == -1) { continue; }
      auto& power = iter.second;
      if (power <= 1) { continue; }
      int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
      Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
      coefficient = coefficient * sqrtRational;
      key = key / (sqrtRational * sqrtRational).numerator();
    }
    answer.content[key] = coefficient;
    return answer;
  }

  QuadraticNumber QuadraticNumber::abs() const
  {
    return ((*this) < QuadraticNumber(0)) ? (-(*this)) : (*this);
  }

  QuadraticNumber QuadraticNumber::operator+() const
  {
    return *this;
  }

  QuadraticNumber QuadraticNumber::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.content) { iter.second = -iter.second; }
    return answer;
  }

  QuadraticNumber QuadraticNumber::operator+(const QuadraticNumber& rhs) const
  {
    std::set<mp> added;
    QuadraticNumber sum;
    for (const auto& iter : content)
    {
      auto radicand = iter.first;
      Rational coeff = iter.second;
      auto primes = radicand.primeFactorization();
      for (const auto& jter : primes)
      {
          auto& factor = jter.first;
          if (factor == -1) { continue; }
          auto& power = jter.second;
          if (power <= 1) { continue; }
          int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
          Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
          coeff = coeff * sqrtRational;
          radicand = radicand / (sqrtRational * sqrtRational).numerator();
      }
      if (rhs.content.find(radicand) != rhs.content.end())
      {
        auto summand = rhs.content.at(radicand);
        if (summand != (-coeff))
        {
          sum.content[radicand] = summand + coeff;
        }
      }
      else { sum.content[radicand] = coeff; }
      added.insert(radicand);
    }
    for (const auto& iter : rhs.content)
    {
      auto radicand = iter.first;
      Rational coeff = iter.second;
      auto primes = radicand.primeFactorization();
      for (const auto& jter : primes)
      {
        auto& factor = jter.first;
        if (factor == -1) { continue; }
        auto& power = jter.second;
        if (power <= 1) { continue; }
        int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
        Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
        coeff = coeff * sqrtRational;
        radicand = radicand / (sqrtRational * sqrtRational).numerator();
      }
      if (added.find(radicand) != added.end()) { continue; }
      sum.content[radicand] = coeff;
    }
    return sum;
  }

  QuadraticNumber QuadraticNumber::operator-(const QuadraticNumber& rhs) const
  {
    return (*this) + (-rhs);
  }

  QuadraticNumber QuadraticNumber::operator*(const QuadraticNumber& rhs) const
  {
    QuadraticNumber product;
    for (const auto& iter : content)
    {
      for (const auto& jter : rhs.content)
      {
        auto summand = sqrt(Rational(iter.first, mp(1)) * Rational(jter.first, mp(1)));
        auto factor = iter.second * jter.second;
        for (auto& kter : summand.content)
        {
          kter.second = kter.second * factor;
        }
        product = product + summand;
      }
    }
    return product;
  }

  QuadraticNumber QuadraticNumber::operator/(const QuadraticNumber& rhs) const
  {
    if (content.empty()) { return *this; }
    if (rhs.content.empty())
    {
      throw std::invalid_argument("Division by zero.");
      return QuadraticNumber();
    }
    std::map<int, mp> index2Root;
    std::map<mp, int> root2Index;
    auto multMatrix = rhs.getMultiplicationMatrix(root2Index, index2Root);
    bool success = false;
    auto multInverse = multMatrix.inverse(success);
    if (!success) { throw std::logic_error("Division failed."); return QuadraticNumber(); }
    auto dim = multMatrix.numRows();
    Matrix<Rational> multVector;
    {
      std::vector<Rational> row(dim, 0);
      row[root2Index[1]] = 1; // root2Index guaranteed to have 1 as a key since sqrt(A)^2 = A * sqrt(1)
      multVector.addRow(row);
      multVector = multVector.transpose();
    }
    multVector = multInverse * multVector;
    QuadraticNumber reciprocal;
    for (int ii = 0; ii < dim; ++ii)
    {
      auto coeff = multVector.at(ii, 0);
      if (coeff == Rational(0, 1)) { continue; }
      reciprocal.content[index2Root[ii]] = coeff;
      //reciprocal = reciprocal + QuadraticNumber::sqrt(Rational(index2Root[ii], 1)) * multVector.at(ii, 0);
    }
    return (*this) * reciprocal;
  }

  QuadraticNumber QuadraticNumber::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    QuadraticNumber answer(Rational(1, 1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return QuadraticNumber(Rational(1, 1)) / answer;
    }
    return answer;
  }

  bool QuadraticNumber::operator==(const QuadraticNumber& rhs) const
  {
    for (const auto& iter : content)
    {
      if (rhs.content.find(iter.first) == rhs.content.end()) { return false; }
      if (rhs.content.at(iter.first) != iter.second) { return false; }
    }
    for (const auto& iter : rhs.content)
    {
      if (content.find(iter.first) == content.end()) { return false; }
      if (content.at(iter.first) != iter.second) { return false; }
    }
    return true;
  }

  bool QuadraticNumber::operator!=(const QuadraticNumber& rhs) const
  {
    return !((*this) == rhs);
  }

  bool QuadraticNumber::operator!=(int rhs) const
  {
    return !((*this) == QuadraticNumber(rhs));
  }

  bool QuadraticNumber::operator<(const QuadraticNumber& rhs) const
  {
    return get() < rhs.get();
  }

  bool QuadraticNumber::operator>(const QuadraticNumber& rhs) const
  {
    return (rhs < (*this));
  }

  bool QuadraticNumber::tryGetCosine(const Rational& input, QuadraticNumber& output)
  {
    if (input < Rational()) { return tryGetCosine(-input, output); }
    if (input == Rational()) { output = QuadraticNumber(Rational(1, 1)); return true; }
    if ((mp(12) % (input.denominator())) != mp(0)) { return false; }
    auto sqrt2 = QuadraticNumber::sqrt(2);
    auto sqrt6 = QuadraticNumber::sqrt(6);
    QuadraticNumber cosPiOver12 = (sqrt6 + sqrt2) * Rational(1, 4);
    QuadraticNumber sinPiOver12 = (sqrt6 - sqrt2) * Rational(1, 4);
    unsigned int power_ = (input.numerator() * (mp(12) / (input.denominator()))).toInt();
    ComplexQuadratic powered = ComplexQuadratic(cosPiOver12, sinPiOver12).pow(power_);
    output = powered.getRe();
    return true;
  }

  bool QuadraticNumber::tryGetSine(const Rational& input, QuadraticNumber& output)
  {
    if (input < Rational())
    {
      QuadraticNumber output0;
      bool answer = tryGetSine(-input, output0);
      if (!answer) { return answer; }
      output = -output0;
      return true;
    }
    if (input == Rational()) { output = QuadraticNumber(); return true; }
    if ((mp(12) % (input.denominator())) != mp(0)) { return false; }
    auto sqrt2 = QuadraticNumber::sqrt(2);
    auto sqrt6 = QuadraticNumber::sqrt(6);
    QuadraticNumber cosPiOver12 = (sqrt6 + sqrt2) * Rational(1, 4);
    QuadraticNumber sinPiOver12 = (sqrt6 - sqrt2) * Rational(1, 4);
    unsigned int power_ = (input.numerator() * (mp(12) / (input.denominator()))).toInt();
    ComplexQuadratic powered = ComplexQuadratic(cosPiOver12, sinPiOver12).pow(power_);
    output = powered.getIm();
    return true;
  }
}
