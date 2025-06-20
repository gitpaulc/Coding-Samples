/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

namespace function_calculator_cs
{
/** \class Multiple-precision integer.
 *
 *  Important for avoiding overflow as calculations become more complicated.
 */
public class mp
{
  /** \brief Stored in reverse place-value as a_0 + a_1 * b + ... + a_p * b^p. */
  List<int> self;
  Boolean negative = false;

  void clean()
  {
    int siz = self.Count;
    int newSiz = siz;
    for (int i = siz - 1; i >= 0; --i)
    {
      if (self[i] != 0) { break; }
      newSiz--;
    }
    if (newSiz < siz)
    {
      self.RemoveRange(newSiz, siz - newSiz);
    }
    if (self.Count == 0) { negative = false; }
  }

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
  static mp gcd(const mp& aa, const mp& bb);
  static mp gcd(const std::vector<mp>& arguments);
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
