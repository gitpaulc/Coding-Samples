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
      if (iter.second == Rational()) { continue; }
      double val = iter.second.get().first;
      double radicand = iter.first.toInt();
      if (iter.first < 0) { throw std::invalid_argument("\nRadicands should be nonnegative."); radicand = -radicand; }
      answer += val * std::sqrt(radicand);
    }
    return { answer, 0.0 };
  }

  int QuadraticNumber::getNumRootsInSum() const
  {
    return (int)content.size();
  }

  std::set<mp> QuadraticNumber::getSummandRoots() const
  {
    std::set<mp> roots;
    for (const auto& iter : content) { roots.insert(iter.first); }
    return roots;
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
    std::vector<mp> numerators;
    numerators.push_back(answer.second);
    for (auto& iter : answer.first.content)
    {
      iter.second = iter.second * Rational(answer.second, mp(1));
      numerators.push_back(iter.second.numerator());
    }
    auto gcd_ = mp::gcd(numerators);
    answer.second = answer.second / gcd_;
    for (auto& iter : answer.first.content)
    {
      iter.second = iter.second * Rational(mp(1), gcd_);
    }
    return answer;
  }

  bool QuadraticNumber::getRational(Rational& self) const
  {
    if (content.size() == 0) { self = Rational(0, 1); return true; }
    if (content.size() > 1) { return false; }
    auto iter = content.find(1);
    if (iter == content.end()) { return false; }
    self = iter->second;
    return true;
  }

  void QuadraticNumber::clean()
  {
    QuadraticNumber answer;
    for (const auto& iter : content)
    {
      if (iter.first == mp(0)) { continue; }
      if (iter.second == Rational(0)) { continue; }
      answer.content[iter.first] = iter.second;
    }
    content = answer.content;
  }

  QuadraticNumber QuadraticNumber::coeffsAbs() const
  {
    QuadraticNumber answer = *this;
    for (auto& iter : answer.content)
    {
      if (iter.second > Rational(0)) { continue; }
      iter.second = -iter.second;
    }
    return answer;
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
    return answer;
  }

  std::set<QuadraticNumber> QuadraticNumber::getIterates0() const
  {
    static std::map<QuadraticNumber, std::set<QuadraticNumber> > sIterates;
    std::set<QuadraticNumber> answer;
    if (content.empty()) { return answer; }
    {
      auto iter = sIterates.find(*this);
      if (iter != sIterates.end())
      {
        return iter->second;
      }
    }
    auto iter = content.begin();
    if (iter == content.end()) { sIterates[*this] = answer; return answer; }
    if (!(iter->second.isInt())) { throw std::logic_error("Use integral quadratic number for intermediate prime factorization."); return answer; }
    mp lim = iter->second.numerator();
    bool isNegative_ = (lim < mp(0));
    mp absLim = lim.abs();
    if (content.size() == 1)
    {
      for (mp ind = mp(0); ind <= absLim; ind = ind + mp(1))
      {
        QuadraticNumber quad;
        auto coeff = (isNegative_ ? (-ind) : (ind));
        quad.content[iter->first] = Rational(coeff, mp(1));
        answer.insert(quad);
      }
      sIterates[*this] = answer;
      return answer;
    }
    std::set<QuadraticNumber> smaller;
    {
      QuadraticNumber other;
      for (const auto& jter : content)
      {
        if (iter->first == jter.first) { continue; }
        other.content[jter.first] = jter.second;
      }
      smaller = other.getIterates0();
    }
    for (mp ind = mp(0); ind <= absLim; ind = ind + mp(1))
    {
      for (const auto& remaining : smaller)
      {
        QuadraticNumber quad;
        auto coeff = (isNegative_ ? (-ind) : (ind));
        quad.content[iter->first] = Rational(coeff, mp(1));
        for (const auto& jter : remaining.content)
        {
          quad.content[jter.first] = jter.second;
        }
        answer.insert(quad);
      }
    }
    sIterates[*this] = answer;
    return answer;
  }

  std::set<QuadraticNumber> QuadraticNumber::getIterates() const
  {
    auto iterates0 = getIterates0();
    std::set<QuadraticNumber> iterates;
    for (const auto& iterate0 : iterates0)
    {
      auto iterate = iterate0;
      iterate.clean();
      if (iterate == QuadraticNumber()) { continue; }
      Rational rationalVal;
      if (iterate.getRational(rationalVal)) { continue; }
      {
        std::vector<mp> coeffs(iterate.content.size());
        int ii = -1;
        for (const auto& iter : iterate.content)
        {
          ++ii;
          coeffs[ii] = iter.second.numerator();
        }
        if ((mp::gcd(coeffs) != mp(1)) && (mp::gcd(coeffs) != mp(-1))) { continue; } // Sieve out multiples of primes.
      }
      iterates.insert(iterate);
    }
    return iterates;
  }

  std::map<QuadraticNumber, int> QuadraticNumber::primeFacIntegral() const
  {
    auto input = *this;
    std::map<QuadraticNumber, int> answer;
    auto lim = input.coeffsAbs();
    std::set<QuadraticNumber> sieved;
    auto iterates = input.getIterates();
    bool foundFactor = false;
    for (const auto& init : iterates)
    {
      for (QuadraticNumber factor = init; factor.coeffsAbs() < lim; factor = factor + init)
      {
        if (sieved.find(factor) != sieved.end()) { continue; }
        sieved.insert(factor);
        auto quotient = input / factor;
        // Is factor a true factor?
        if (iterates.find(quotient) == iterates.end()) { continue; }
        if (factor == input) { continue; }
        foundFactor = true;
        if (answer.find(factor) == answer.end())
        {
          answer[factor] = 1;
        }
        else { answer[factor] = answer[factor] + 1; }
        auto others = quotient.primeFacIntegral();
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
    if (!foundFactor)
    {
      answer[input] = 1;
    }
    return answer;
  }

  std::string QuadraticNumber::print(bool useParentheses) const
  {
    std::stringstream strm;
    int count = -1;
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
    auto returnStr = strm.str();
    if (returnStr.empty()) { returnStr = "0"; }
    if (useParentheses) { returnStr = std::string("(") + returnStr + ")"; }
    return returnStr;
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

  std::pair<QuadraticNumber, QuadraticNumber> QuadraticNumber::separateSquaredPart() const
  {
    std::pair<QuadraticNumber, QuadraticNumber> answer;
    if ((*this) == QuadraticNumber())
    {
      answer.first = QuadraticNumber();
      answer.second = QuadraticNumber();
      return answer;
    }
    answer.first = QuadraticNumber(Rational(1));
    answer.second = QuadraticNumber(Rational(1));
    auto factors = primeFactorization();
    for (const auto& iter : factors)
    {
      auto prim = iter.first;
      int expon = iter.second;
      if (expon < 0)
      {
        prim = QuadraticNumber(Rational(1)) / prim;
        expon = -expon;
      }
      if ((expon % 2) == 1)
      {
        answer.first = answer.first * prim.pow((expon - 1) / 2);
        answer.second = answer.second * prim;
        continue;
      }
      answer.first = answer.first * prim.pow(expon / 2);
    }
    return answer;
  }

  int QuadraticNumber::simpleSquareRoot(QuadraticNumber& sqrt1, QuadraticNumber& sqrt2) const
  {
    int numRoots = 0;
    if (content.size() > 2) { return numRoots; } // Restrict attention to simple expressions.
    auto iter = content.find(mp(1));
    if (iter == content.end()) { return numRoots; }
    if (content.size() == 1)
    {
      if ((iter->second) < Rational()) { return numRoots; }
      sqrt1 = QuadraticNumber::sqrt(iter->second);
      numRoots = 1;
      if (iter->second != 0)
      {
        sqrt2 = -sqrt1;
        numRoots = 2;
      }
      return numRoots;
    }
    auto& uu = iter->second;
    auto vv = uu;
    Rational dd;
    for (const auto& jter : content)
    {
      if (iter->first == jter.first) { continue; }
      vv = jter.second;
      dd = Rational(jter.first, mp(1));
      break;
    }
    Rational two(2);
    Rational a2, a2_other;
    {
      Rational radical;
      Rational radicand = uu * uu - vv * vv * dd;
      if (radicand < Rational()) { return false; }
      auto rad = QuadraticNumber::sqrt(radicand);
      bool radIsRational = rad.getRational(radical);
      if (!radIsRational) { return numRoots; }
      a2 = (uu + radical) / two;
      if (a2 < Rational()) { return numRoots; }
      numRoots = 1;
      if (radical <= uu)
      {
        a2_other = (uu - radical) / two;
        numRoots = 2;
      }
    }
    QuadraticNumber aa, aa_other;
    aa = QuadraticNumber::sqrt(a2);
    auto bb = QuadraticNumber(vv) / (aa + aa);
    auto sqrtDD = QuadraticNumber::sqrt(dd);
    auto sqrt1_ = aa + bb * sqrtDD;
    //if (sqrt1_ * sqrt1_ != (*this)) { return 0; }
    sqrt1 = sqrt1_;
    if (numRoots >= 2)
    {
      aa_other = QuadraticNumber::sqrt(a2_other);
      auto bb_other = QuadraticNumber(vv) / (aa_other + aa_other);
      auto sqrt2_ = aa_other + bb_other * sqrtDD;
      //if (sqrt2_ * sqrt2_ != (*this)) { return 1; }
      sqrt2 = sqrt2_;
    }
    return numRoots;
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
      auto jter = rhs.content.find(iter.first);
      if (jter == rhs.content.end()) { return false; }
      if ((jter->second) != iter.second) { return false; }
    }
    for (const auto& iter : rhs.content)
    {
      auto jter = content.find(iter.first);
      if (jter == content.end()) { return false; }
      if ((jter->second) != iter.second) { return false; }
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

  void QuadraticNumber::getLowerUpperBounds(const Rational& radicand, const unsigned int& numIterations, Rational& lower, Rational& upper)
  {
    if (radicand < Rational())
    {
      throw std::invalid_argument("Radicand must be nonnegative.");
    }
    struct TableEntry
    {
      unsigned numIters = 0;
      Rational lower, upper;
    };
    static std::map<Rational, TableEntry> cachedBounds;
    {
      auto iter = cachedBounds.find(radicand);
      if (iter != cachedBounds.end())
      {
        auto& entry = iter->second;
        if (entry.lower == entry.upper)
        {
          if (entry.numIters < numIterations)
          {
            entry.numIters = numIterations;
          }
          lower = entry.lower;
          upper = entry.upper;
          return;
        }
        if (entry.numIters >= numIterations)
        {
          lower = entry.lower;
          upper = entry.upper;
          return;
        }
      }
    }
    Rational one_(1);
    if (radicand == one_)
    {
      lower = one_;
      upper = one_;
      return;
    }
    Rational lower_, upper_;
    if (radicand > one_)
    {
      lower_ = one_;
      upper_ = radicand;
    }
    else // if (radicand < one_)
    {
      lower_ = radicand;
      upper_ = one_;
    }
    for (unsigned int nn = 1; nn < numIterations; ++nn)
    {
      auto bisection = (lower_ + upper_) * Rational(1, 2);
      auto comparer = bisection * bisection;
      if (comparer == radicand)
      {
        lower = bisection;
        upper = bisection;
        TableEntry entry;
        entry.numIters = numIterations;
        entry.lower = lower;
        entry.upper = upper;
        cachedBounds[radicand] = entry;
        return;
      }
      if (comparer < radicand)
      {
        lower_ = bisection;
        continue;
      }
      // if (comparer > radicand)
      {
        upper_ = bisection;
        continue;
      }
    }
    lower = lower_;
    upper = upper_;
    TableEntry entry;
    entry.numIters = numIterations;
    entry.lower = lower;
    entry.upper = upper;
    cachedBounds[radicand] = entry;
  }

  void QuadraticNumber::getLowerUpperBounds(const unsigned int& numIterations, Rational& lower, Rational& upper) const
  {
    Rational lower_, upper_;
    Rational ll, uu;
    mp one_(1);
    for (const auto& iter : content)
    {
      getLowerUpperBounds(Rational(iter.first, one_), numIterations, ll, uu);
      ll = ll * iter.second;
      uu = uu * iter.second;
      if (ll <= uu)
      {
        lower_ = lower_ + ll;
        upper_ = upper_ + uu;
        continue;
      }
      lower_ = lower_ + uu;
      upper_ = upper_ + ll;
    }
    lower = lower_;
    upper = upper_;
  }

  bool QuadraticNumber::operator<(const QuadraticNumber& rhs) const
  {
    if ((*this) == rhs) { return false; }
    Rational l0, u0, l1, u1;
    for (unsigned nn = 5; true; nn += 5)
    {
      getLowerUpperBounds(nn, l0, u0);
      rhs.getLowerUpperBounds(nn, l1, u1);
      if (u0 < l1) { return true; }
      if (u1 < l0) { return false; }
    }
    return false;
  }

  bool QuadraticNumber::operator>(const QuadraticNumber& rhs) const
  {
    return (rhs < (*this));
  }

  std::map<QuadraticNumber, int> QuadraticNumber::primeFactorization() const
  {
    auto input = *this;
    std::map<QuadraticNumber, int> answer;
    {
      Rational inputAsRational;
      if (input.getRational(inputAsRational))
      {
        auto factors = inputAsRational.primeFactorization();
        for (const auto& iter : factors)
        {
          answer[QuadraticNumber(Rational(iter.first, mp(1)))] = iter.second;
        }
        return answer;
      }
    }
    if (input < QuadraticNumber()) { answer[QuadraticNumber(Rational(mp(-1), mp(1)))] = 1; input = -input; }
    {
      auto asIntegral = input.factorAsIntegral();
      input = asIntegral.first;
      mp gcd_ = mp(1);
      bool factorWithGcd = false;
      if (factorWithGcd)
      {
        std::vector<mp> coeffs;
        for (const auto& iter : input.content)
        {
          coeffs.push_back(iter.second.numerator());
        }
        gcd_ = mp::gcd(coeffs);
        input = input * QuadraticNumber(Rational(mp(1), gcd_));
      }
      auto factors = Rational(gcd_, asIntegral.second).primeFactorization();
      for (const auto& iter : factors)
      {
        if (iter.first == mp(1)) { continue; }
        answer[QuadraticNumber(Rational(iter.first, mp(1)))] = iter.second;
      }
    }
    const auto fac = input.primeFacIntegral();
    for (const auto& iter : fac)
    {
      auto jter = answer.find(iter.first);
      if (jter == answer.end())
      {
        answer[iter.first] = iter.second;
        continue;
      }
      jter->second = jter->second + iter.second;
    }
    return answer;
  }

  std::string QuadraticNumber::printFactors(bool useParentheses) const
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
      strm << iter.first.print(true);
      strm << "^" << iter.second;
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  bool QuadraticNumber::tryGetCosine(const Rational& input, QuadraticNumber& output)
  {
    BiquadraticNumber answer;
    bool success = BiquadraticNumber::tryGetCosine(input, answer);
    if (!success) { return false; }
    QuadraticNumber answerQ;
    success = answer.getAsQuadratic(answerQ);
    if (!success) { return false; }
    output = answerQ;
    return true;
  }

  bool QuadraticNumber::tryGetSine(const Rational& input, QuadraticNumber& output)
  {
    BiquadraticNumber answer;
    bool success = BiquadraticNumber::tryGetSine(input, answer);
    if (!success) { return false; }
    QuadraticNumber answerQ;
    success = answer.getAsQuadratic(answerQ);
    if (!success) { return false; }
    output = answerQ;
    return true;
  }
}
