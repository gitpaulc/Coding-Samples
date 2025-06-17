/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef MP_INTEGER_H
#define MP_INTEGER_H

#include <map>
#include <ostream>
#include <vector>

namespace FunctionalCalculator
{
/** \class Multiple-precision integer.
 *
 *  Important for avoiding overflow as calculations become more complicated.
 */
class mp
{
  /** \brief Stored in reverse place-value as a_0 + a_1 * b + ... + a_p * b^p. */
  std::vector<int> self;
  bool negative = false;
  void clean();
  const static int digPow;
  const static int limit;

  mp division(const mp& rhs, mp& remainder) const;

public:
  mp(int value = 0);
  mp(const long long& value);

  int getDigit(int i) const;
  void setDigit(int i, int val);
  int numDigits() const;
  int toInt() const;
  /** \return { a, b } where a is the maximal number such that this integer == a * a * b */
  std::pair<mp, mp> separateSquaredPart() const;

  mp operator+() const;
  mp operator-() const;
  mp operator+(const mp& rhs) const;
  mp operator-(const mp& rhs) const;
  mp operator*(const mp& rhs) const;
  mp operator/(const mp& rhs) const;
  mp operator%(const mp& rhs) const;
  mp abs() const;
  static mp gcd(const mp& aa, const mp& bb);
  mp pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const mp& rhs) const;
  bool operator!=(const mp& rhs) const;
  bool operator<(const mp& rhs) const;
  bool operator>(const mp& rhs) const;
  bool operator<=(const mp& rhs) const;
  bool operator>=(const mp& rhs) const;

  /** \return n! / ((n - k)! * k!) */
  static mp binomialCoeff(int n, int k);
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  std::map<mp, int> primeFactorization() const;

  friend std::ostream& operator<<(std::ostream& strm, const mp& mpIn);
};

std::ostream& operator<<(std::ostream& strm, const mp& mpIn);

}

#endif //def MP_INTEGER_H
