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
      if (iter.first < 0) { throw std::runtime_error("\nStill need to implement complex numbers."); radicand = -radicand; }
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
    std::set<int> added;
    QuadraticNumber sum;
    for (const auto& iter : content)
    {
      int radicand = iter.first;
      Rational coeff = iter.second;
      auto primes = Rational::primeFactorization(radicand);
      for (const auto& jter : primes)
      {
          auto& factor = jter.first;
          if (factor == -1) { continue; }
          auto& power = jter.second;
          if (power <= 1) { continue; }
          int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
          Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
          coeff = coeff * sqrtRational;
          radicand /= (sqrtRational * sqrtRational).numerator();
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
      int radicand = iter.first;
      Rational coeff = iter.second;
      auto primes = Rational::primeFactorization(radicand);
      for (const auto& jter : primes)
      {
        auto& factor = jter.first;
        if (factor == -1) { continue; }
        auto& power = jter.second;
        if (power <= 1) { continue; }
        int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
        Rational sqrtRational = Rational(factor, 1).pow(coeffPow);
        coeff = coeff * sqrtRational;
        radicand /= (sqrtRational * sqrtRational).numerator();
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
        bool iterNegative = (iter.second < 0);
        bool jterNegative = (jter.second < 0);
        auto summand = sqrt(Rational(iter.first, 1) * Rational(jter.first, 1) *
          iter.second * iter.second * jter.second * jter.second);
        if (iterNegative && !jterNegative) { summand = -summand; }
        else if (jterNegative && !iterNegative) { summand = -summand; }
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
    QuadraticNumber quotientNumer = Rational(1, 1);
    QuadraticNumber quotientDenom = rhs;
    while (quotientDenom.content.size() > 1)
    {
      int radicand = 1;
      Rational coeff;
      for (const auto& iter : quotientDenom.content)
      {
        if (iter.first == 1) { continue; }
        radicand = iter.first;
        coeff = iter.second;
        break;
      }
      if (radicand == 1) { break; }
      QuadraticNumber diff;
      auto sqrtTerm = QuadraticNumber::sqrt(radicand) * (-coeff);
      quotientNumer = quotientNumer * (quotientDenom + (sqrtTerm * Rational(2, 1)));
      diff = quotientDenom + sqrtTerm;
      quotientDenom = (diff * diff) - (coeff * coeff * radicand);
    }
    quotientNumer = quotientNumer * quotientDenom;
    quotientDenom = quotientDenom * quotientDenom;
    Rational rationalDenom;
    bool success = quotientDenom.getRational(rationalDenom);
    if (!success) { throw std::logic_error("Division failed."); }
    return (*this) * quotientNumer * (Rational(1, 1) / rationalDenom);
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
}
