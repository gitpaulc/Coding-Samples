/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "rational.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  Rational::Rational(int nn, int dd)
  {
    *this = Rational(mp(nn), mp(dd));
  }

  Rational::Rational(const mp& nn, const mp& dd)
  {
    if (dd == mp(0))
    {
      throw std::invalid_argument("Division by zero.");
    }
    else if (nn == mp(0))
    {
      num = nn;
      denom = mp(1);
    }
    else
    {
      mp gcd_ = gcd(nn, dd);
      num = nn; denom = dd;
      if (gcd_ != 0)
      {
        num = num / gcd_;
        denom = denom / gcd_;
      }
      if (denom < 0)
      {
        num = num * mp(-1);
        denom = denom * mp(-1);
      }
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

  mp Rational::denominator() const { return denom; }
  mp Rational::numerator() const { return num; }

  mp Rational::gcd(mp aa, mp bb)
  {
    if ((aa == bb) || (bb == 0)) { return (aa > 0) ? aa : (-aa); }
    if (aa == 0) { return (bb > 0) ? bb : (-bb); }
    mp abs_a = (aa > mp(0)) ? aa : -aa;
    mp abs_b = (bb > mp(0)) ? bb : -bb;
    //if (bb != 0) { return gcd(bb, aa % bb); }
    for (mp safety_counter = mp(2) * abs_a + mp(2) * abs_b; bb != 0; safety_counter = safety_counter - mp(1))
    {
      if (safety_counter <= 0) { break; }
      mp aa_old = aa;
      mp bb_old = bb;
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

  Rational Rational::operator*(const mp& rhs) const
  {
      return Rational(num * rhs, denom);
  }

  Rational Rational::operator/(const Rational& rhs) const
  {
    if (rhs.num == 0)
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return Rational(num * rhs.denom, denom * rhs.num);
  }

  Rational Rational::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    Rational answer(1, 1);
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return Rational(answer.denom, answer.num);
    }
    return answer;
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

  std::pair<double, double> Rational::get() const
  {
    double nn = (double)(num.toInt());
    double dd = (double)(denom.toInt());
    return { nn / dd, 0.0 };
  }

  std::string Rational::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    if (num == 0) { strm << num; }
    else if (denom == 1) { strm << num; }
    else { strm << num << " / " << denom; }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  std::map<mp, int> Rational::primeFactorization(mp input)
  {
    if (input * input <= 1)
    {
        std::map<mp, int> answer;
        answer[input] = 1;
        return answer;
    }
    std::map<mp, int> answer;
    if (input < 0) { answer[-1] = 1; input = -input; }
    mp lim = input + 1;
    std::set<mp> sieved;
    bool foundFactor = false;
    for (mp init = 2; init < lim; init = init + 1)
    {
      for (mp factor = init; factor < lim; factor = factor + init)
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

  std::map<mp, int> Rational::primeFactorization() const
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
    std::map<mp, int> answer;
    int countFactors = (int)numFactors.size();
    for (auto& iter : numFactors)
    {
      mp base = iter.first;
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

  std::string Rational::printFactors(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
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
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }
}
