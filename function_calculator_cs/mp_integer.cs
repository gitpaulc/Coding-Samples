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
  private List<int> self = null;
  private Boolean negative = false;

  private static int digPow = 6;
  private static int intPow(int baseInt, int p)
  {
    if (p < 0) { throw new System.Exception("Exponent must be nonnegative."); }
    int answer = 1;
    for (int ii = 0; ii < p; ++ii) { answer *= baseInt; }
    return answer;
  }
  static int limit = intPow(10, digPow);
  private void clean()
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

  private mp division(in mp rhs, ref mp remainder)
  {
    mp zero_ = new mp(0);
    mp ten_ = new mp(10);
    if (rhs == zero_)
    {
      if (this == zero_) { remainder = new mp(0);  return new mp(1); }
      throw new System.Exception("Division by zero.");
    }
    if (this == zero_) { remainder = new mp(0);  return new mp(0); }
    if (negative && rhs.negative) { return (-this).division(-rhs, ref remainder); }
    if (negative) { return -((-(this)).division(rhs, ref remainder)); }
    if (rhs.negative) { return -(division(-rhs, ref remainder)); }

    mp dividend = new mp(this);
    mp quotient = new mp(0);
    mp prevDividend = dividend;
    while (rhs <= dividend)
    {
      int numOfDigits = dividend.numDigits();
      int remainingDigits = numOfDigits - 1;
      mp miniDividend = new mp(dividend.getDigit(remainingDigits));
      for (int ii = 2; rhs > miniDividend; --ii)
      {
        --remainingDigits;
        miniDividend = miniDividend * ten_;
        miniDividend = miniDividend + (new mp(dividend.getDigit(remainingDigits)));
        if (remainingDigits == 0) { break; }
      }
      int bestDigit = 1;
      while (rhs * (bestDigit + 1) < miniDividend)
      {
        if (bestDigit == 9) { break; }
        ++bestDigit;
      }
      mp factor = (new mp(bestDigit)) * ten_.pow(remainingDigits);
      quotient = quotient + factor;
      dividend = dividend - factor * rhs;
      if (dividend >= prevDividend) { break; } // Should never happen.
      prevDividend = dividend;
    }
    remainder = dividend;
    return quotient;
  }

  static mp()
  {
  }

  public mp(int value = 0)
  {
    self = new List<int>();
    if (value != 0)
    {
      if (value < 0) { negative = true; value = -value; }
      while (value >= limit)
      {
        self.Add(value % limit);
        value = value / limit;
      }
      self.Add(value);
    }
  }

  public mp(in long value)
  {
    self = new List<int>();
    if (value != 0)
    {
      long lim = limit;
      long val = value;
      if (value < 0) { negative = true; val = -value; }
      while (val >= lim)
      {
        self.Add((int)(val % lim));
        val = val / lim;
      }
      self.Add((int)val);
    }
  }

  public mp(in mp other)
  {
    self = new List<int>(other.self);
    negative = other.negative;
  }

  public int getDigit(int i);
  public void setDigit(int i, int val);
  public int numDigits();
  public int toInt();
  static mp gcd(in mp aa, in mp bb);
  static mp gcd(in List<mp> arguments);
  /** \return { a, b } where a is the maximal number such that this integer == a * a * b */
  KeyValuePair<mp, mp> separateSquaredPart();

  public static mp operator+(in mp body) { return; }
  public static mp operator-(in mp body) { return; }
  public static mp operator+(in mp body, in mp rhs) { return; }
  public static mp operator-(in mp body, in mp rhs) { return; }
  public static mp operator*(in mp body, in mp rhs) { return; }
  public static mp operator*(in mp body, int rhs) { return; }
  public static mp operator/(in mp body, in mp rhs) { return; }
  public static mp operator%(in mp body, in mp rhs) { return; }
  mp abs() const;
  mp pow(int p) const; /**< `return` The p'th power of the number. */
  public static bool operator==(in mp body, in mp rhs) { return new mp(); }
  public static bool operator!=(in mp body, in mp rhs) { return new mp(); }
  public static bool operator<(in mp body, in mp rhs) { return new mp(); }
  public static bool operator>(in mp body, in mp rhs) { return new mp(); }
  public static bool operator<=(in mp body, in mp rhs) { return new mp(); }
  public static bool operator>=(in mp body, in mp rhs) { return new mp(); }

  /** \return n! / ((n - k)! * k!) */
  static mp binomialCoeff(int n, int k);
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  std::map<mp, int> primeFactorization() const;

  friend std::ostream& operator<<(std::ostream& strm, const mp& mpIn);
};

std::ostream& operator<<(std::ostream& strm, const mp& mpIn);

}

#endif //def MP_INTEGER_H
