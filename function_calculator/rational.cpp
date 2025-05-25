/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "rational.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  Rational::Rational(int nn, int dd)
  {
    if (dd == 0)
    {
      throw std::invalid_argument("Division by zero.");
    }
    int gcd_ = gcd(nn, dd);
    num = nn; denom = dd;
    if (gcd_ != 0)
    {
      num = num / gcd_;
      denom = denom / gcd_;
    }
    if (denom < 0)
    {
      num *= -1;
      denom *= -1;
    }
  }

  Rational::Rational(const Rational& rhs)
  {
    num = rhs.num;
    denom = rhs.denom;
  }

  Rational::Rational(Rational&& rhs) noexcept
  {
    num = rhs.num;
    denom = rhs.denom;
  }

  Rational& Rational::operator=(const Rational& rhs)
  {
    if (this != &rhs)
    {
      num = rhs.num;
      denom = rhs.denom;
    }
    return *this;
  }

  Rational& Rational::operator=(Rational&& rhs) noexcept
  {
    if (this != &rhs)
    {
      num = rhs.num;
      denom = rhs.denom;
    }
    return *this;
  }

  int Rational::denominator() const { return denom; }
  int Rational::numerator() const { return num; }

  int Rational::gcd(int aa, int bb)
  {
    if ((aa == bb) || (bb == 0)) { return (aa > 0) ? aa : (-aa); }
    if (aa == 0) { return (bb > 0) ? bb : (-bb); }
    int abs_a = (aa > 0) ? aa : -aa;
    int abs_b = (bb > 0) ? bb : -bb;
    //if (bb != 0) { return gcd(bb, aa % bb); }
    for (int safety_counter = 2 * abs_a + 2 * abs_b; bb != 0; --safety_counter)
    {
      if (safety_counter <= 0) { break; }
      int aa_old = aa;
      int bb_old = bb;
      aa = bb_old;
      bb = aa_old % bb_old;
    }
    if (aa < 0) { return -aa; }
    return aa;
  }

  Rational Rational::operator+() const
  {
    return *this;
  }

  Rational Rational::operator-() const
  {
    return Rational(-num, denom);
  }

  Rational Rational::operator+(const Rational& rhs) const
  {
    return Rational(num * rhs.denom + rhs.num * denom, denom * rhs.denom);
  }

  Rational Rational::operator-(const Rational& rhs) const
  {
    return ((*this) + (-rhs));
  }

  Rational Rational::operator*(const Rational& rhs) const
  {
    return Rational(num * rhs.num, denom * rhs.denom);
  }

  Rational Rational::operator/(const Rational& rhs) const
  {
    if (rhs.num == 0)
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return Rational(num * rhs.denom, denom * rhs.num);
  }

  bool Rational::operator==(const Rational& rhs) const
  {
    if (rhs.num != num) { return false; }
    if (rhs.denom != denom) { return false; }
    return true;
  }

  bool Rational::operator!=(const Rational& rhs) const
  {
    if (*this == rhs) { return false; }
    return true;
  }

  bool Rational::operator<(const Rational& rhs) const
  {
    if (num * rhs.denom < denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator>(const Rational& rhs) const
  {
    if (num * rhs.denom > denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator<=(const Rational& rhs) const
  {
    if (num * rhs.denom <= denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator>=(const Rational& rhs) const
  {
    if (num * rhs.denom >= denom * rhs.num) { return true; }
    return false;
  }

  double Rational::get() const
  {
    double nn = (double)num;
    double dd = (double)denom;
    return nn / dd;
  }

  std::string Rational::print() const
  {
    std::stringstream strm;
    strm << "(";
    if (num == 0) { strm << num; }
    else if (denom == 1) { strm << num; }
    else { strm << num << " / " << denom; }
    strm << ")";
    return strm.str();
  }

  std::map<int, int> Rational::primeFactorization(int input)
  {
    if (input * input <= 1)
    {
        std::map<int, int> answer;
        answer[input] = 1;
        return answer;
    }
    std::map<int, int> answer;
    if (input < 0) { answer[-1] = 1; input = -input; }
    int lim = input + 1;
    std::set<int> sieved;
    bool foundFactor = false;
    for (int init = 2; init < lim; ++init)
    {
      for (int factor = init; factor < lim; factor += init)
      {
        if (sieved.find(factor) != sieved.end()) { continue; }
        sieved.insert(factor);
        if (input % factor != 0) { continue; }
        foundFactor = true;
        if (factor == input)
        {
          if (answer.find(factor) == answer.end())
          {
            answer[factor] = 1;
            break;
          }
          answer[factor] = answer[factor] + 1;
          break;
        }
        if (answer.find(factor) == answer.end())
        {
          answer[factor] = 1;
        }
        else { answer[factor] = answer[factor] + 1; }
        auto others = primeFactorization(input / factor);
        for (auto& iter : others)
        {
          if (answer.find(iter.first) == answer.end())
          {
            answer[iter.first] = iter.second;
            continue;
          }
          answer[iter.first] += iter.second;
        }
        break;
      }
      if (foundFactor) { break; }
    }
    return answer;
  }

  std::map<int, int> Rational::primeFactorization() const
  {
    auto numFactors = primeFactorization(num);
    auto denomFactors = primeFactorization(denom);
    for (auto& iter : denomFactors)
    {
      if (iter.first == 1) { continue; }
      if (numFactors.find(iter.first) == numFactors.end())
      {
        numFactors[iter.first] = -iter.second;
        continue;
      }
      numFactors[iter.first] -= iter.second;
    }
    std::map<int, int> answer;
    int countFactors = (int)numFactors.size();
    for (auto& iter : numFactors)
    {
      int base = iter.first;
      int power = iter.second;
      if (base == -1)
      {
        if (power < 0) { power = -power; }
        power = (power % 2);
        if ((countFactors == 1) && (power == 0)) { base = 1; power = 1; }
      }
      if ((base == 1) && (countFactors > 0)) { power = 0; }
      if (power == 0)
      {
        if (countFactors == 1) { base = 1; power = 1; }
        else { continue; }
      }
      answer[base] = power;
    }
    return answer;
  }

  std::string Rational::printFactors() const
  {
    std::stringstream strm;
    strm << "(";
    auto factors = primeFactorization();
    int count = -1;
    for (auto& iter : factors)
    {
      if (iter.second == 0) { continue; }
      ++count;
      if (count > 0) { strm << " * "; }
      if (iter.first < 0) { strm << "("; }
      strm << iter.first;
      if (iter.first < 0) { strm << ")"; }
      strm << "^" << iter.second;
    }
    strm << ")";
    return strm.str();
  }
}
