/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "mp_integer.h"

#include <set>
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

  int mp::getDigit(int i) const
  {
    if (i < 0) { throw std::invalid_argument("Index must be nonnegative."); }
    int j = i % digPow;
    int ind = i / digPow;
    if (ind >= self.size()) { return 0; }
    int current = self[ind];
    current = current / intPow(10, j);
    return current % 10;
  }

  void mp::setDigit(int i, int val)
  {
    if (i < 0) { throw std::invalid_argument("Index must be nonnegative."); }
    if (val < 0) { throw std::invalid_argument("Digit must be between 0 and 9 inclusive."); }
    if (val >= 10) { throw std::invalid_argument("Digit must be between 0 and 9 inclusive."); }
    int j = i % digPow;
    int ind = i / digPow;
    bool shouldClean = (val == 0);
    if (ind >= self.size())
    {
      int oldSize = (int)self.size();
      self.resize(ind + 1);
      shouldClean = true;
      for (int k = oldSize; k < (ind + 1); ++k) { self[k] = 0; }
    }
    int& current = self[ind];
    auto powJ = intPow(10, j);
    auto powJ1 = 10 * powJ;
    auto right = current % powJ;
    int summand = val * powJ + right;
    auto left = ((j + 1) == digPow) ? 0 : (current / powJ1) * powJ1;
    current = left + summand;
    if (shouldClean) { clean(); }
  }

  int mp::numDigits() const
  {
    if (self.empty()) { return 0; }
    int ind = (int)(self.size()) - 1;
    int current = self[ind];
    int best = 0;
    for (int i = 0; i < digPow; ++i)
    {
      if ((current % 10) != 0) { best = i + 1; }
      current = current / 10;
    }
    return best + digPow * (int)(self.size() - 1);
  }

  int mp::toInt() const
  {
    if (self.empty()) { return 0; }
    return negative ? (-self[0]) : self[0];
  }

  mp mp::gcd(const mp& aa, const mp& bb)
  {
    auto aa0 = aa; auto bb0 = bb;
    if ((aa0 == bb0) || (bb0 == 0)) { return (aa0 > 0) ? aa0 : (-aa0); }
    if (aa0 == 0) { return (bb0 > 0) ? bb0 : (-bb0); }
    mp abs_a = (aa0 > mp(0)) ? aa0 : -aa0;
    mp abs_b = (bb0 > mp(0)) ? bb0 : -bb0;
    //if (bb0 != 0) { return gcd(bb0, aa0 % bb0); }
    for (mp safety_counter = mp(2) * abs_a + mp(2) * abs_b; bb0 != 0; safety_counter = safety_counter - mp(1))
    {
      if (safety_counter <= 0) { break; }
      mp aa0_old = aa0;
      mp bb0_old = bb0;
      aa0 = bb0_old;
      bb0 = aa0_old % bb0_old;
    }
    if (aa0 < 0) { return -aa0; }
    return aa0;
  }

  mp mp::gcd(const std::vector<mp>& arguments)
  {
    if (arguments.empty()) { throw std::logic_error("Cannot take gcd of no integers."); return mp(0); }
    int numArgs = (int)arguments.size();
    mp answer = arguments[0];
    for (int ii = 1; ii < numArgs; ++ii)
    {
      answer = gcd(answer, arguments[ii]);
    }
    return answer;
  }

  std::pair<mp, mp> mp::separateSquaredPart() const
  {
    std::pair<mp, mp> answer;
    if ((*this) == mp(0))
    {
      answer.first = mp(0);
      answer.second = mp(0);
      return answer;
    }
    answer.first = mp(1);
    answer.second = mp(1);
    auto factors = primeFactorization();
    for (const auto& iter : factors)
    {
      if ((iter.second % 2) == 1)
      {
        if (answer.first < 0)
        {
          answer.second = answer.second * iter.first;
          continue;
        }
        answer.first = answer.first * iter.first.pow((iter.second - 1) / 2);
        answer.second = answer.second * iter.first;
        continue;
      }
      if (answer.first < 0) { continue; }
      answer.first = answer.first * iter.first.pow(iter.second / 2);
    }
    return answer;
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
    if (negative && rhs.negative) { return -((-rhs) + (-(*this))); }
    if (rhs.negative) { return ((*this) - (-rhs)); }
    if (negative) { return (rhs - (-(*this))); }
    if (rhs.self.size() > self.size()) { return (rhs + (*this)); }
    mp answer = *this;
    const auto rhsSize = rhs.self.size();

    int ii = -1;
    int carry = 0;
    long long summandA = 0;
    long long summandB = 0;
    long long lim = limit;
    for (auto& iter : rhs.self)
    {
      ++ii;
      if (ii >= rhsSize) { continue; }
      summandA = answer.self[ii];
      summandB = iter;
      auto sum = summandA + summandB + carry;
      carry = 0;
      if (sum >= (long long)lim)
      {
        carry = 1;
        sum = sum % lim;
      }
      answer.self[ii] = (int)sum;
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
    // Both are nonnegative...

    const int numOfDigits = numDigits();
    const int numRhsDigits = rhs.numDigits();

    // Negative answer:
    if (numRhsDigits > numOfDigits) { return -(rhs - (*this)); }
    if (numRhsDigits == numOfDigits)
    {
      for (int ii = numOfDigits - 1; ii >= 0; --ii)
      {
        int digit = rhs.getDigit(ii);
        int subFrom = getDigit(ii);
        if (digit > subFrom) { return -(rhs - (*this)); }
        if (digit < subFrom) { break; }
      }
    }

    //Nonnegative number:
    mp answer;
    answer.self.resize(self.size());
    answer.negative = false;
    auto from = *this;
    for (int ii = 0; ii < numOfDigits; ++ii)
    {
      int digit = rhs.getDigit(ii);
      int subFrom = from.getDigit(ii);
      if (digit > subFrom)
      {
        if (ii == (numOfDigits - 1)) { throw std::logic_error("Bad subtraction."); }
        else
        {
          int jj = ii + 1;
          int current = from.getDigit(jj);
          while (current == 0)
          {
            from.setDigit(jj, 9);
            ++jj;
            current = from.getDigit(jj);
          }
          from.setDigit(jj, current - 1);
          subFrom += 10;
        }
      }
      answer.setDigit(ii, subFrom - digit);
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
        const int answerSize = (int)answer.self.size();
        if (kk >= answerSize)
        {
          answer.self.resize(kk + 1);
          for (int ll = answerSize; ll < kk + 1; ++ll) { answer.self[ll] = 0; }
        }
        long long product = (long long)(self[ii]) * (long long)(rhs.self[jj]) + carry;
        carry = (int)(product / lim);
        answer.self[kk] = answer.self[kk] + ((int)(product % lim));
      }
      if (carry > 0)
      {
        auto kk = ii + sizB;
        const int answerSize = (int)answer.self.size();
        if (kk >= answerSize)
        {
          answer.self.resize(kk + 1);
          for (int ll = answerSize; ll < kk + 1; ++ll) { answer.self[ll] = 0; }
        }
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

    auto dividend = *this;
    mp quotient = mp(0);
    auto prevDividend = dividend;
    while (rhs <= dividend)
    {
      const int numOfDigits = dividend.numDigits();
      int remainingDigits = numOfDigits - 1;
      mp miniDividend = dividend.getDigit(remainingDigits);
      for (int ii = 2; rhs > miniDividend; --ii)
      {
        --remainingDigits;
        miniDividend = miniDividend * mp(10);
        miniDividend = miniDividend + mp(dividend.getDigit(remainingDigits));
        if (remainingDigits == 0) { break; }
      }
      int bestDigit = 1;
      while (rhs * (bestDigit + 1) < miniDividend)
      {
        if (bestDigit == 9) { break; }
        ++bestDigit;
      }
      mp factor = mp(bestDigit) * mp(10).pow(remainingDigits);
      quotient = quotient + factor;
      dividend = dividend - factor * rhs;
      if (dividend >= prevDividend) { break; } // Should never happen.
      prevDividend = dividend;
    }
    remainder = dividend;
    return quotient;
  }

  mp mp::abs() const
  {
    mp answer = *this;
    answer.negative = false;
    return answer;
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

  mp mp::binomialCoeff(int n, int k)
  {
    if (n < 0) { return 0; }
    if (k < 0) { return 0; }
    if (k > n) { return 0; }
    static std::map<std::pair<int, int>, mp> binoms;
    if (binoms.empty()) { binoms[{0, 0}] = mp(1); }
    auto iter = binoms.find({ n, k });
    if (iter != binoms.end()) { return iter->second; }
    auto answer = binomialCoeff(n - 1, k) + binomialCoeff(n - 1, k - 1);
    binoms[{n, k}] = answer;
    return answer;
  }

  std::map<mp, int> mp::primeFactorization() const
  {
    auto input = *this;
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
        auto quotient = input / factor;
        if (quotient.abs() >= input.abs()) { throw std::logic_error("Bad prime factorization."); break; } // Should never happen.
        auto others = quotient.primeFactorization();
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
