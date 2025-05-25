/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "quadratic_number.h"

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

  double QuadraticNumber::get() const
  {
    double answer = 0.0;
    for (const auto& iter : content)
    {
      double val = iter.second.get();
      if (val == 0) { continue; }
      double radicand = iter.first;
      if (iter.first < 0) { throw std::exception("\nStill need to implement complex numbers."); radicand = -radicand; }
      answer += val * std::sqrt(radicand);
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
      int radicand = iter.first;
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

  void QuadraticNumber::factorSquares()
  {
    auto contentOld = content;
    content = std::map<int, Rational>();
    for (auto& iter : contentOld)
    {
      auto summand = sqrt(Rational(iter.first, 1) * iter.second * iter.second);
      *this = *this + summand;
    }
  }

  QuadraticNumber QuadraticNumber::sqrt(const Rational& radicand)
  {
    QuadraticNumber answer;
    Rational coefficient(1, radicand.denominator());
    int key = radicand.numerator() * radicand.denominator();
    auto primes = Rational::primeFactorization(key);
    for (const auto& iter : primes)
    {
      auto& factor = iter.first;
      if (factor == -1) { continue; }
      auto& power = iter.second;
      if (power <= 1) { continue; }
      int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
      Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
      coefficient = coefficient * sqrtRational;
      key /= (sqrtRational * sqrtRational).numerator();
    }
    answer.content[key] = coefficient;
    return answer;
  }

  QuadraticNumber QuadraticNumber::operator+(const QuadraticNumber& rhs) const
  {
    std::set<int> added;
    QuadraticNumber sum;
    for (const auto& iter : content)
    {
      if (rhs.content.find(iter.first) != rhs.content.end())
      {
        auto summand = rhs.content.at(iter.first);
        if (summand != (- iter.second))
        {
          sum.content[iter.first] = summand + iter.second;
        }
      }
      else { sum.content[iter.first] = iter.second; }
      added.insert(iter.first);
    }
    for (const auto& iter : rhs.content)
    {
      if (added.find(iter.first) == added.end()) { continue; }
      sum.content[iter.first] = iter.second;
    }
    return sum;
  }
}
