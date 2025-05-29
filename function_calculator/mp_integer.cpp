/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "mp_integer.h"

#include <stdexcept>

namespace FunctionalCalculator
{
  int mp::limit = 65536;

  void mp::clean()
  {
    mp answer;
    bool nonzero = false;
    int siz = (int)self.size();
    int firstNonzero = -1;
    for (int i = 0; i < siz; ++i)
    {
      if (!nonzero)
      {
        if (self[i] == 0) { continue; }
        firstNonzero = i;
        answer.self.resize(siz - firstNonzero);
      }
      nonzero = true;
      answer.self[i - firstNonzero] = self[i];
    }
    self = answer.self;
    if (self.empty()) { negative = false; }
  }

  mp::mp(int value)
  {
    if (value != 0)
    {
      if (value < 0) { negative = true; value = -value; }
      while (value >= limit)
      {
        self.push_back(value % limit);
        value = value / limit;
      }
      self.push_back(value);
    }
  }

  mp::mp(const long long& value)
  {
    if (value != 0)
    {
      long long lim = limit;
      auto val = value;
      if (value < 0) { negative = true; val = -value; }
      while (val >= lim)
      {
        self.push_back((int)(val % lim));
        val = val / lim;
      }
      self.push_back((int)val);
    }
  }

  mp mp::operator+() const
  {
    return *this;
  }

  mp mp::operator-() const
  {
    auto answer = *this;
    if (answer.self.empty()) { return answer; }
    answer.negative = !answer.negative;
    return answer;
  }

  mp mp::operator+(const mp& rhs) const
  {
    if (negative && rhs.negative) { return -(rhs + (*this)); }
    if (rhs.negative) { return ((*this) - (-rhs)); }
    if (negative) { return (rhs - (-(*this))); }
    if (rhs.degree() > degree()) { return (rhs + (*this)); }
    mp answer = *this;
    const auto rhsDeg = rhs.degree();

    int ii = -1;
    for (auto& iter : rhs.self)
    {
      ++ii;
      if (ii > rhsDeg) { continue; }
      answer.self[ii] = answer.self[ii] + iter;
    }
    answer.clean();
    return answer;
  }

  mp mp::operator-(const mp& rhs) const
  {
    if (rhs.self.empty()) { return (*this); }
    if (self.empty()) { return -rhs; }
    if (rhs.negative && (!negative)) { return (*this) + (-rhs); }
    if (rhs.negative && negative) { return ((-rhs) - (-(*this))); }
    // rhs is nonnegative:
    if (negative) { return -((-(*this)) + rhs); }
    // both are nonnegative:
    auto deg = degree();
    auto rhsDeg = rhs.degree();
    // Negative number:
    if (rhsDeg > deg) { return -(rhs - (*this)); }
    if (rhsDeg == deg)
    {
      if (rhs.self[rhsDeg] > self[deg]) { return -(rhs - (*this)); }
    }
    //Nonnegative number:
    mp answer;
    answer.self.resize(deg + 1);
    answer.negative = false;
    auto from = *this;
    for (int ii = deg; ii >= 0; --ii)
    {
      int digit = rhs.self[ii];
      int subFrom = from.self[ii];
      if (digit > subFrom)
      {
        if (ii == 0) { throw std::logic_error("Bad subtraction."); }
        else
        {
          from.self[ii - 1] = from.self[ii - 1] - 1;
          subFrom += limit;
        }
      }
      answer.self[ii] = subFrom - digit;
    }
    answer.clean();
    return answer;
  }

  mp mp::operator*(const mp& rhs) const
  {
    mp answer(0);
    if (self.empty()) { return answer; }
    if (rhs.self.empty()) { return answer; }
    answer.negative = (negative || rhs.negative) && !(negative && rhs.negative);
    answer.self.resize((degree() + 1) * (rhs.degree() + 1), 0);

    int ii = -1;
    for (const auto& iter : self)
    {
      ++ii;
      int jj = -1;
      for (const auto& jter : rhs.self)
      {
        ++jj;
        auto kk = ii + jj;
        answer.self[kk] = answer.self[kk] + iter * jter;
      }
    }
    return answer;
  }

  int mp::degree() const
  {
    if (self.empty()) { return 0; }
    return (int)(self.size() - 1);
  }

  mp mp::division(const mp& rhs, mp& remainder) const
  {
    if (rhs == mp(0))
    {
      if ((*this) == mp(0)) { remainder = mp(0);  return mp(1); }
      throw std::invalid_argument("Division by zero.");
      remainder = mp(0);  return mp(1);
    }
    auto rhsDegree = rhs.degree();
    if (rhsDegree == 0)
    {
        mp quotient(self /rhs.self[0]);
      remainder = mp(0);
      return quotient;
    }
    auto dividend = *this;
    auto divDegree = dividend.degree();
    mp quotient = mp(0);
    for (int prevDegree = divDegree; rhsDegree <= divDegree;)
    {
      auto monomial = mp(rhs.self.at(rhsDegree) / dividend.self.at(divDegree), divDegree - rhsDegree);
      quotient = quotient + monomial;
      auto product = rhs * monomial;
      if (product == dividend) { remainder = mp(0); return quotient; }
      dividend = dividend - product;
      prevDegree = divDegree;
      divDegree = dividend.degree();
      if (prevDegree <= divDegree) { break; } // Should never happen.
    }
    remainder = dividend;
    return quotient;
  }

  mp mp::gcd(const mp& aa, const mp& bb)
  {
    if ((aa == bb) || (aa == mp(0))) { return bb; }
    if (bb == mp(0)) { return aa; }
    auto aPoly = aa;
    auto bPoly = bb;
    while (bPoly != mp(0))
    {
      auto aa_old = aPoly;
      auto bb_old = bPoly;
      aPoly = bb_old;
      auto quotient = aa_old.division(bb_old, bPoly);
    }
    if (aPoly == mp(0)) { return aPoly; }
    auto aPolyDegree = aPoly.degree();
    auto coeff = aPoly.self[aPolyDegree];
    for (auto& iter : aPoly.self)
    {
      iter = iter / coeff;
    }
    return aPoly;
  }

  mp mp::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { throw std::invalid_argument("Exponent must be nonnegative."); }
    mp answer;
    answer.self[0] = 1;
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    return answer;
  }

  bool mp::operator==(const mp& rhs) const
  {
    auto diff = (*this) - rhs;
    for (const auto& iter : diff.self)
    {
      if (iter != 0) { return false; }
    }
    return true;
  }

  bool mp::operator!=(const mp& rhs) const
  {
    return !((*this) == rhs);
  }

  bool mp::operator<(const mp& rhs) const
  {
    auto diff = (*this) - rhs;
    return diff.negative;
  }
}
