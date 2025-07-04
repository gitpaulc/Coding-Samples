/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#include "biquadratic_number.h"

#include "complex_quadratic.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  BiquadraticNumber::BiquadraticNumber(const Rational& number) :
    BiquadraticNumber(QuadraticNumber(number))
  {
  }

  BiquadraticNumber::BiquadraticNumber(const QuadraticNumber& number)
  {
    if (number != 0)
    {
      QuadraticNumber one_(Rational(1));
      content[one_] = number;
    }
  }

  std::pair<double, double> BiquadraticNumber::get() const
  {
    double answer = 0.0;
    for (const auto& iter : content)
    {
      if (iter.second == QuadraticNumber()) { continue; }
      double val = iter.second.get().first;
      double radicand = iter.first.get().first;
      if (radicand < 0) { throw std::invalid_argument("\nRadicands should be nonnegative."); radicand = -radicand; }
      answer += val * std::sqrt(radicand);
    }
    return { answer, 0.0 };
  }

  std::pair<BiquadraticNumber, mp> BiquadraticNumber::factorAsIntegral() const
  {
    std::pair<BiquadraticNumber, mp> answer;
    std::vector<QuadraticNumber> keys, gammas;
    std::vector<mp> iotas, js;
    mp bigDenom(1);
    for (const auto& iter : content)
    {
      auto rr = iter.first.factorAsIntegral();
      auto qq = iter.second.factorAsIntegral();
      keys.push_back(rr.first * QuadraticNumber(Rational(rr.second, mp(1))));
      gammas.push_back(qq.first);
      iotas.push_back(qq.second);
      js.push_back(rr.second);
      bigDenom = bigDenom * (rr.second * qq.second);
    }
    std::vector<mp> coeffs(iotas.size() + 1);
    const int numCoeffs = (int)iotas.size();
    for (int ii = 0; ii < numCoeffs; ++ii)
    {
      coeffs[ii] = bigDenom / (iotas[ii] * js[ii]);
    }
    coeffs[numCoeffs] = bigDenom;
    const mp gcd_ = mp::gcd(coeffs);
    for (auto& coeff : coeffs) { coeff = coeff / gcd_; }
    answer.second = coeffs[numCoeffs];
    for (int ii = 0; ii < numCoeffs; ++ii)
    {
      answer.first.content[keys[ii]] = gammas[ii] * QuadraticNumber(Rational(coeffs[ii], mp(1)));
    }
    return answer;
  }

  bool BiquadraticNumber::getAsQuadratic(QuadraticNumber& self) const
  {
    if (content.size() == 0) { self = Rational(0, 1); return true; }
    if (content.size() > 1) { return false; }
    auto iter = content.find(QuadraticNumber(Rational(1)));
    if (iter == content.end()) { return false; }
    self = iter->second;
    return true;
  }

  bool BiquadraticNumber::getRational(Rational& self) const
  {
    QuadraticNumber quad;
    if (!getAsQuadratic(quad)) { return false; }
    Rational rr;
    if (!quad.getRational(rr)) { return false; }
    self = rr;
    return true;
  }

  Matrix<QuadraticNumber> BiquadraticNumber::getMultiplicationMatrix(std::map<QuadraticNumber, int>& root2Index, std::map<int, QuadraticNumber>& index2Root) const
  {
    root2Index = std::map<QuadraticNumber, int>();
    index2Root = std::map<int, QuadraticNumber>();
    {
      std::set<QuadraticNumber> rootsSoFar;
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
    auto answer = Matrix<QuadraticNumber>::zeroMatrix(dimMatrix);
    for (const auto& iter : content)
    {
      Matrix<QuadraticNumber> summand;
      const auto& radA = iter.first;
      for (int ii = 0; ii < dimMatrix; ++ii)
      {
        std::vector<QuadraticNumber> row(dimMatrix, Rational(0));
        const auto& radB = index2Root[ii];
        auto root = radA * radB;
        auto sqrtSplit = root.separateSquaredPart();
        row[root2Index[sqrtSplit.second]] = sqrtSplit.first;
        summand.addRow(row);
      }
      answer = answer + summand.transpose() * iter.second;
    }
    return answer;
  }

  std::string BiquadraticNumber::print(bool useParentheses) const
  {
    std::stringstream strm;
    QuadraticNumber one_(Rational(1));
    int count = -1;
    if (useParentheses) { strm << "("; }
    if (content.size() == 0) { strm << "0"; }
    for (const auto& iter : content)
    {
      auto val = iter.second;
      if (val == QuadraticNumber()) { continue; }
      ++count;
      if (count > 0)
      {
        if (val == QuadraticNumber()) { strm << " + "; }
        else if (QuadraticNumber() < val) { strm << " + "; }
        else
        {
          val = -val;
          strm << " - ";
        }
      }
      bool coeffIsOne = (val == one_);
      auto radicand = iter.first;
      bool printCoeffParents = (radicand != one_);
      if ((radicand == one_) || (!coeffIsOne)) { strm << val.print(printCoeffParents); }
      if (radicand == one_) { continue; }
      bool complex = false;
      if (radicand < QuadraticNumber()) { radicand = -radicand; complex = true; }
      if (!coeffIsOne) { strm << " * "; }
      if (!complex || (radicand != 1))
      {
        strm << "Sqrt(" << radicand.print() << ")";
        if (complex) { strm << " * "; }
      }
      if (complex) { strm << "i"; }
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  BiquadraticNumber BiquadraticNumber::sqrt(const Rational& radicand)
  {
    return sqrt(QuadraticNumber(radicand));
  }

  BiquadraticNumber BiquadraticNumber::sqrt(const QuadraticNumber& radicand)
  {
    {
      Rational ratio;
      bool radIsRational = radicand.getRational(ratio);
      if (radIsRational)
      {
        BiquadraticNumber answer;
        auto sqrtNum = QuadraticNumber::sqrt(ratio);
        answer.content[QuadraticNumber(Rational(1))] = sqrtNum;
        return answer;
      }
    }
    BiquadraticNumber answer;
    auto squaredPart = radicand.separateSquaredPart();
    answer.content[squaredPart.second] = squaredPart.first;
    return answer;
  }

  BiquadraticNumber BiquadraticNumber::abs() const
  {
    return ((*this) < BiquadraticNumber(0)) ? (-(*this)) : (*this);
  }

  BiquadraticNumber BiquadraticNumber::operator+() const
  {
    return *this;
  }

  BiquadraticNumber BiquadraticNumber::operator-() const
  {
    auto answer = *this;
    for (auto& iter : answer.content) { iter.second = -iter.second; }
    return answer;
  }

  BiquadraticNumber BiquadraticNumber::operator+(const BiquadraticNumber& rhs) const
  {
    std::set<QuadraticNumber> added;
    BiquadraticNumber sum;
    for (const auto& iter : content)
    {
      auto radicand = iter.first;
      QuadraticNumber coeff = iter.second;
      auto primes = radicand.primeFactorization();
      for (const auto& jter : primes)
      {
        auto& factor = jter.first;
        if (factor == QuadraticNumber(Rational(-1))) { continue; }
        auto& power = jter.second;
        if (power <= 1) { continue; }
        int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
        auto sqrtRational = factor.pow(coeffPow);
        coeff = coeff * sqrtRational;
        radicand = radicand / (sqrtRational * sqrtRational);
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
      QuadraticNumber coeff = iter.second;
      auto primes = radicand.primeFactorization();
      for (const auto& jter : primes)
      {
        auto& factor = jter.first;
        if (factor == QuadraticNumber(Rational(-1))) { continue; }
        auto& power = jter.second;
        if (power <= 1) { continue; }
        int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
        auto sqrtRational = factor.pow(coeffPow);
        coeff = coeff * sqrtRational;
        radicand = radicand / (sqrtRational * sqrtRational);
      }
      if (added.find(radicand) != added.end()) { continue; }
      sum.content[radicand] = coeff;
    }
    return sum;
  }

  BiquadraticNumber BiquadraticNumber::operator-(const BiquadraticNumber& rhs) const
  {
    return (*this) + (-rhs);
  }

  BiquadraticNumber BiquadraticNumber::operator*(const BiquadraticNumber& rhs) const
  {
    BiquadraticNumber product;
    for (const auto& iter : content)
    {
      for (const auto& jter : rhs.content)
      {
        std::pair<QuadraticNumber, QuadraticNumber> sqPart;
        bool shouldComputeSqPart = true;
        if ((iter.first != QuadraticNumber()) && (jter.first != QuadraticNumber()))
        {
          auto quotient = jter.first / iter.first;
          Rational ratio;
          if (quotient.getRational(ratio))
          {
            if (ratio > Rational())
            {
              shouldComputeSqPart = false;
              // Setting sqPart to follow separateSquaredPart() method.
              sqPart.first = iter.first * QuadraticNumber::sqrt(ratio);
              sqPart.second = QuadraticNumber(Rational(1)); // See below.
            }
          }
        }
        if (shouldComputeSqPart)
        {
          sqPart = (iter.first * jter.first).separateSquaredPart();
        }
        auto& key = sqPart.second;
        auto kter = product.content.find(key);
        if (kter == product.content.end())
        {
          product.content[key] = iter.second * jter.second * sqPart.first;
          continue;
        }
        kter->second = kter->second + iter.second * jter.second * sqPart.first;
      }
    }

    BiquadraticNumber answer;
    for (const auto& iter : product.content)
    {
      if ((iter.first.getNumRootsInSum() == 2) && (iter.second.getNumRootsInSum() <= 2))
      {
        auto roots_1 = iter.first.getSummandRoots();
        auto roots_2 = iter.second.getSummandRoots();
        bool shouldChangeKey = false;
        for (const auto& root : roots_1)
        {
          if (root == mp(1)) { shouldChangeKey = true; break; }
        }
        for (const auto& root : roots_2)
        {
          if (!shouldChangeKey) { break; }
          if (roots_2.find(root) == roots_2.end()) { shouldChangeKey = false; }
        }
        if (shouldChangeKey)
        {
          auto newKey = iter.second * iter.second * iter.first;
          QuadraticNumber newVal(Rational(1));
          if (iter.second < QuadraticNumber()) { newVal = -newVal; }
          auto jter = answer.content.find(newKey);
          if (jter == answer.content.end())
          {
            answer.content[newKey] = newVal;
            continue;
          }
          jter->second = jter->second + newVal;
          continue;
        }
      }
      auto jter = answer.content.find(iter.first);
      if (jter == answer.content.end())
      {
        answer.content[iter.first] = iter.second; continue;
      }
      jter->second = jter->second + iter.second;
    }
    return answer;
  }

  BiquadraticNumber BiquadraticNumber::operator/(const BiquadraticNumber& rhs) const
  {
    if (content.empty()) { return *this; }
    if (rhs.content.empty())
    {
      throw std::invalid_argument("Division by zero.");
      return BiquadraticNumber();
    }
    std::map<int, QuadraticNumber> index2Root;
    std::map<QuadraticNumber, int> root2Index;
    auto multMatrix = rhs.getMultiplicationMatrix(root2Index, index2Root);
    bool success = false;
    auto multInverse = multMatrix.inverse(success);
    if (!success) { throw std::logic_error("Division failed."); return BiquadraticNumber(); }
    auto dim = multMatrix.numRows();
    Matrix<QuadraticNumber> multVector;
    {
      std::vector<QuadraticNumber> row(dim, Rational());
      QuadraticNumber one_(Rational(1));
      row[root2Index[one_]] = one_; // root2Index guaranteed to have 1 as a key since sqrt(A)^2 = A * sqrt(1)
      multVector.addRow(row);
      multVector = multVector.transpose();
    }
    multVector = multInverse * multVector;
    BiquadraticNumber reciprocal;
    for (int ii = 0; ii < dim; ++ii)
    {
      auto coeff = multVector.at(ii, 0);
      if (coeff == Rational(0, 1)) { continue; }
      reciprocal.content[index2Root[ii]] = coeff;
      //reciprocal = reciprocal + BiquadraticNumber::sqrt(Rational(index2Root[ii], 1)) * multVector.at(ii, 0);
    }
    return (*this) * reciprocal;
  }

  BiquadraticNumber BiquadraticNumber::pow(int p) const
  {
    bool isNeg = (p < 0);
    if (isNeg) { p = -p; }
    BiquadraticNumber answer(Rational(1, 1));
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (*this);
    }
    if (isNeg)
    {
      return BiquadraticNumber(Rational(1, 1)) / answer;
    }
    return answer;
  }

  bool BiquadraticNumber::operator==(const BiquadraticNumber& rhs) const
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

  bool BiquadraticNumber::operator!=(const BiquadraticNumber& rhs) const
  {
    return !((*this) == rhs);
  }

  bool BiquadraticNumber::operator!=(int rhs) const
  {
    return !((*this) == BiquadraticNumber(rhs));
  }

  bool BiquadraticNumber::operator<(const BiquadraticNumber& rhs) const
  {
    return get() < rhs.get();
  }

  bool BiquadraticNumber::operator>(const BiquadraticNumber& rhs) const
  {
    return (rhs < (*this));
  }

  namespace // anonymous
  {
    std::map<Rational, BiquadraticNumber> cosineValues;
    std::map<Rational, BiquadraticNumber> sineValues;
  }

  bool BiquadraticNumber::tryGetCosine(const Rational& input, BiquadraticNumber& output)
  {
    if (input < Rational()) { return tryGetCosine(-input, output); }
    if (input == Rational()) { output = BiquadraticNumber(Rational(1, 1)); return true; }
    {
      auto iter = cosineValues.find(input);
      if (iter != cosineValues.end()) { output = iter->second; return true; }
    }
    std::set<mp> admissibles = { mp(60) };
    bool isAdmissible = false;
    for (const auto& admissible : admissibles)
    {
      if ((admissible % (input.denominator())) == mp(0))
      {
        isAdmissible = true; break;
      }
    }
    if (!isAdmissible) { return false; }
    if (input.numerator() > input.denominator() / 2)
    {
      Rational input1(input.denominator() - input.numerator(), input.denominator());
      BiquadraticNumber cosInput1;
      auto success = tryGetCosine(input1, cosInput1);
      if (!success) { return false; }
      output = -cosInput1;
      cosineValues[input] = output;
      return true;
    }
    mp numeratorHalf1 = input.numerator() / 2;
    mp numeratorHalf2 = input.numerator() - numeratorHalf1;
    if ((numeratorHalf1 < input.numerator()) && (numeratorHalf2 < input.numerator()))
    {
      if (input.numerator() == mp(4))
      {
        isAdmissible = true;
      }
      Rational input1(numeratorHalf1, input.denominator());
      Rational input2(numeratorHalf2, input.denominator());
      BiquadraticNumber cosInput1, cosInput2, sinInput1, sinInput2;
      auto success = tryGetCosine(input1, cosInput1);
      if (!success) { return false; }
      if (numeratorHalf1 == numeratorHalf2)
      {
        output = cosInput1 * cosInput1 * BiquadraticNumber(QuadraticNumber(Rational(2)))
          - BiquadraticNumber(QuadraticNumber(Rational(1)));
        cosineValues[input] = output;
        return true;
      }
      success = tryGetCosine(input2, cosInput2);
      if (!success) { return false; }
      success = tryGetSine(input1, sinInput1);
      if (!success) { return false; }
      success = tryGetSine(input2, sinInput2);
      if (!success) { return false; }
      output = cosInput1 * cosInput2 - sinInput1 * sinInput2;
      cosineValues[input] = output;
      return true;
    }
    if ((mp(12) % (input.denominator())) == mp(0))
    {
      auto sqrt2 = BiquadraticNumber::sqrt(2);
      auto sqrt6 = BiquadraticNumber::sqrt(6);
      BiquadraticNumber cosPiOver12 = (sqrt6 + sqrt2) * Rational(1, 4);
      BiquadraticNumber sinPiOver12 = (sqrt6 - sqrt2) * Rational(1, 4);
      unsigned int power_ = (input.numerator() * (mp(12) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver12, sinPiOver12).pow(power_);
      output = powered.getRe();
      cosineValues[input] = output;
      return true;
    }
    if ((mp(5) % (input.denominator())) == mp(0))
    {
      auto sqrt5 = QuadraticNumber::sqrt(5);
      auto four = QuadraticNumber(Rational(4));
      auto s2 = (QuadraticNumber(Rational(5)) - sqrt5) / (four + four);
      BiquadraticNumber cosPiOver5 = (QuadraticNumber(Rational(1)) + sqrt5) / four;
      BiquadraticNumber sinPiOver5 = BiquadraticNumber::sqrt(s2);
      unsigned int power_ = (input.numerator() * (mp(5) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver5, sinPiOver5).pow(power_);
      output = powered.getRe();
      cosineValues[input] = output;
      return true;
    }
    if ((mp(60) % (input.denominator())) == mp(0))
    {
      auto sqrt2 = QuadraticNumber::sqrt(2);
      auto sqrt5 = QuadraticNumber::sqrt(5);
      auto sqrt6 = QuadraticNumber::sqrt(6);
      auto four = QuadraticNumber(Rational(4));
      auto s2 = (QuadraticNumber(Rational(5)) + sqrt5) / (four + four);
      BiquadraticNumber cos3PiOver5((QuadraticNumber(Rational(1)) - sqrt5) / four);
      BiquadraticNumber sin3PiOver5 = BiquadraticNumber::sqrt(s2);
      BiquadraticNumber cos7PiOver12((sqrt2 - sqrt6) / four);
      BiquadraticNumber sin7PiOver12((sqrt2 + sqrt6) / four);
      auto cosPiOver60 = cos3PiOver5 * cos7PiOver12 + sin3PiOver5 * sin7PiOver12;
      auto sinPiOver60 = sin3PiOver5 * cos7PiOver12 - cos3PiOver5 * sin7PiOver12;
      unsigned int power_ = (input.numerator() * (mp(60) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver60, sinPiOver60).pow(power_);
      output = powered.getRe();
      cosineValues[input] = output;
      return true;
    }
    return false;
  }

  bool BiquadraticNumber::tryGetSine(const Rational& input, BiquadraticNumber& output)
  {
    if (input < Rational())
    {
      BiquadraticNumber output0;
      bool answer = tryGetSine(-input, output0);
      if (!answer) { return answer; }
      output = -output0;
      return true;
    }
    if (input == Rational()) { output = BiquadraticNumber(); return true; }
    {
      auto iter = sineValues.find(input);
      if (iter != sineValues.end()) { output = iter->second; return true; }
    }
    std::set<mp> admissibles = { mp(60) };
    bool isAdmissible = false;
    for (const auto& admissible : admissibles)
    {
      if ((admissible % (input.denominator())) == mp(0))
      {
        isAdmissible = true; break;
      }
    }
    if (!isAdmissible) { return false; }
    if (input.numerator() > input.denominator() / 2)
    {
      Rational input1(input.denominator() - input.numerator(), input.denominator());
      BiquadraticNumber sinInput1;
      auto success = tryGetSine(input1, sinInput1);
      if (!success) { return false; }
      output = sinInput1;
      sineValues[input] = output;
      return true;
    }
    mp numeratorHalf1 = input.numerator() / 2;
    mp numeratorHalf2 = input.numerator() - numeratorHalf1;
    if ((numeratorHalf1 < input.numerator()) && (numeratorHalf2 < input.numerator()))
    {
      Rational input1(numeratorHalf1, input.denominator());
      Rational input2(numeratorHalf2, input.denominator());
      BiquadraticNumber cosInput1, cosInput2, sinInput1, sinInput2;
      auto success = tryGetCosine(input1, cosInput1);
      if (!success) { return false; }
      success = tryGetSine(input1, sinInput1);
      if (!success) { return false; }
      if (numeratorHalf1 == numeratorHalf2)
      {
        output = sinInput1 * cosInput1 * BiquadraticNumber(QuadraticNumber(Rational(2)));
        sineValues[input] = output;
        return true;
      }
      success = tryGetCosine(input2, cosInput2);
      if (!success) { return false; }
      success = tryGetSine(input2, sinInput2);
      if (!success) { return false; }
      output = sinInput1 * cosInput2 + cosInput1 * sinInput2;
      cosineValues[input] = output;
      return true;
    }
    if ((mp(12) % (input.denominator())) == mp(0))
    {
      auto sqrt2 = BiquadraticNumber::sqrt(2);
      auto sqrt6 = BiquadraticNumber::sqrt(6);
      BiquadraticNumber cosPiOver12 = (sqrt6 + sqrt2) * Rational(1, 4);
      BiquadraticNumber sinPiOver12 = (sqrt6 - sqrt2) * Rational(1, 4);
      unsigned int power_ = (input.numerator() * (mp(12) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver12, sinPiOver12).pow(power_);
      output = powered.getIm();
      sineValues[input] = output;
      return true;
    }
    if ((mp(5) % (input.denominator())) == mp(0))
    {
      auto sqrt5 = QuadraticNumber::sqrt(5);
      auto four = QuadraticNumber(Rational(4));
      auto s2 = (QuadraticNumber(Rational(5)) - sqrt5) / (four + four);
      BiquadraticNumber cosPiOver5 = (QuadraticNumber(Rational(1)) + sqrt5) / four;
      BiquadraticNumber sinPiOver5 = BiquadraticNumber::sqrt(s2);
      unsigned int power_ = (input.numerator() * (mp(5) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver5, sinPiOver5).pow(power_);
      output = powered.getIm();
      sineValues[input] = output;
      return true;
    }
    if ((mp(60) % (input.denominator())) == mp(0))
    {
      auto sqrt2 = QuadraticNumber::sqrt(2);
      auto sqrt5 = QuadraticNumber::sqrt(5);
      auto sqrt6 = QuadraticNumber::sqrt(6);
      auto four = QuadraticNumber(Rational(4));
      auto s2 = (QuadraticNumber(Rational(5)) + sqrt5) / (four + four);
      BiquadraticNumber cos3PiOver5((QuadraticNumber(Rational(1)) - sqrt5) / four);
      BiquadraticNumber sin3PiOver5 = BiquadraticNumber::sqrt(s2);
      BiquadraticNumber cos7PiOver12((sqrt2 - sqrt6) / four);
      BiquadraticNumber sin7PiOver12((sqrt2 + sqrt6) / four);
      auto cosPiOver60 = cos3PiOver5 * cos7PiOver12 + sin3PiOver5 * sin7PiOver12;
      auto sinPiOver60 = sin3PiOver5 * cos7PiOver12 - cos3PiOver5 * sin7PiOver12;
      unsigned int power_ = (input.numerator() * (mp(60) / (input.denominator()))).toInt();
      ComplexQuadratic powered = ComplexQuadratic(cosPiOver60, sinPiOver60).pow(power_);
      output = powered.getIm();
      sineValues[input] = output;
      return true;
    }
    return false;
  }
}
