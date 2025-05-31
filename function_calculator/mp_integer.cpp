/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "mp_integer.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  const int mp::digPow = 6;
  int intPow(const int base, const int p)
  {
    if (p < 0) { throw std::invalid_argument("Exponent must be nonnegative."); }
    int answer = 1;
    for (int ii = 0; ii < p; ++ii) { answer *= base; }
    return answer;
  }
  const int mp::limit = intPow(10, digPow);

  void mp::clean()
  {
    int siz = (int)self.size();
    int newSiz = siz;
    for (int i = siz - 1; i >= 0; --i)
    {
      if (self[i] != 0) { break; }
      newSiz--;
    }
    self.resize(newSiz);
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
    int carry = 0;
    long long summandA = 0;
    long long summandB = 0;
    long long lim = limit;
    for (auto& iter : rhs.self)
    {
      ++ii;
      if (ii > rhsDeg) { continue; }
      summandA = answer.self[ii];
      summandB = iter;
      auto sum = summandA + summandB + carry;
      carry = 0;
      if (sum >= (long long)lim)
      {
        carry = 1;
        sum = sum % lim;
      }
      answer.self[ii] = sum;
    }
    if (carry > 0)
    {
      answer.self.push_back(carry);
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

    int carry = 0;
    int sizA = (int)self.size();
    int sizB = (int)rhs.self.size();
    answer.self = std::vector<int>(std::max(sizA, sizB) * std::max(sizA, sizB) + 1, 0);
    long long lim = (long long)limit;

    for (int ii = 0; ii < sizA; ++ii)
    {
      int carry = 0;
      for (int jj = 0; jj < sizB; ++jj)
      {
        auto kk = ii + jj;
        if (kk >= answer.self.size()) { answer.self.resize(kk + 1); answer.self[kk] = 0; }
        long long product = (long long)(self[ii]) * (long long)(rhs.self[jj]) + carry;
        carry = product / lim;
        answer.self[kk] = answer.self[kk] + ((int)(product % lim));
      }
      if (carry > 0)
      {
        auto kk = ii + sizB;
        if (kk >= answer.self.size()) { answer.self.resize(kk + 1); answer.self[kk] = 0; }
        answer.self[kk] = answer.self[kk] + carry;
      }
    }
    answer.clean();
    return answer;
  }

  mp mp::operator/(const mp& rhs) const
  {
    mp remainder;
    auto quotient = division(rhs, remainder);
    return quotient;
  }

  mp mp::operator%(const mp& rhs) const
  {
    mp remainder;
    auto quotient = division(rhs, remainder);
    return remainder;
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
    if ((*this) == mp(0)) { remainder = mp(0);  return mp(0); }
    if (negative && rhs.negative) { return (-(*this)).division(-rhs, remainder); }
    if (negative) { return -((-(*this)).division(rhs, remainder)); }
    if (rhs.negative) { return -(division(-rhs, remainder)); }

    auto rhsDegree = rhs.degree();
    auto dividend = *this;
    mp quotient = mp(0);
    auto prevDividend = dividend;
    while (rhs >= dividend)
    {
      auto divDegree = dividend.degree();
      auto degreeDiff = divDegree - rhsDegree;
      int multiplier = (dividend.self[divDegree] / rhs.self[rhsDegree]) + 1;
      auto factor = mp(multiplier) * mp(limit).pow(degreeDiff);
      auto product = factor * rhs;
      while (product > dividend)
      {
        if (multiplier == 0)
        {
          multiplier = limit - 1;
          degreeDiff--;
        }
        factor = mp(multiplier) * mp(limit).pow(degreeDiff);
        product = factor * rhs;
      }
      quotient = quotient + factor;
      dividend = dividend - factor * rhs;
      if (dividend >= prevDividend) { break; } // Should never happen.
      prevDividend = dividend;
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
    mp answer = 1;
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

  bool mp::operator>(const mp& rhs) const
  {
    return (rhs < (*this));
  }

  bool mp::operator<=(const mp& rhs) const
  {
    if ((*this) == rhs) { return true; }
    return ((*this) < rhs);
  }

  bool mp::operator>=(const mp& rhs) const
  {
    return (rhs <= (*this));
  }

  std::ostream& operator<<(std::ostream& strm, const mp& mpIn)
  {
    if (mpIn == 0) { return strm << "0"; }
    if (mpIn.negative) { return strm << "-" << (-mpIn); }
    std::stringstream reversed;
    int digitCount = 0;
    int nn = (int)(mpIn.self.size());
    for (int ii = 0; ii < nn; ++ii)
    {
      auto element = mpIn.self[ii];
      for (int jj = 0; jj < mp::digPow; ++jj)
      {
        reversed << (element % 10);
        ++digitCount;
        element = element / 10;
        if ((element == 0) && (ii == nn - 1)) { break; }
        if ((digitCount % 3) == 0) { reversed << ","; }
      }
    }
    auto rev = reversed.str();
    nn = (int)rev.size();
    for (int ii = 0; ii < nn; ++ii)
    {
      strm << rev[nn - ii - 1];
    }
    return strm;
  }
}
