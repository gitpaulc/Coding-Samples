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

  public static mp zero() { return new mp(0); }

  private mp division(in mp rhs, ref mp remainder)
  {
    mp zero_ = zero();
    mp ten_ = new mp(10);
    if (rhs == zero_)
    {
      if (this == zero_) { remainder = new mp(0);  return new mp(1); }
      throw new System.Exception("Division by zero.");
    }
    if (this == zero_) { remainder = new mp(0);  return zero(); }
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

  public int getDigit(int i)
  {
    if (i < 0) { throw new System.Exception("Index must be nonnegative."); }
    int j = i % digPow;
    int ind = i / digPow;
    if (ind >= self.Count) { return 0; }
    int current = self[ind];
    current = current / intPow(10, j);
    return current % 10;
  }

  public void setDigit(int i, int val)
  {
    if (i < 0) { throw new System.Exception("Index must be nonnegative."); }
    if (val < 0) { throw new System.Exception("Digit must be between 0 and 9 inclusive."); }
    if (val >= 10) { throw new System.Exception("Digit must be between 0 and 9 inclusive."); }
    int j = i % digPow;
    int ind = i / digPow;
    bool shouldClean = (val == 0);
    if (ind >= self.Count)
    {
      int oldSize = (int)self.Count;
      self.AddRange(new List<int>(ind + 1 - oldSize)); //self.resize(ind + 1);
      shouldClean = true;
      for (int k = oldSize; k < (ind + 1); ++k) { self[k] = 0; }
    }
    var powJ = intPow(10, j);
    var powJ1 = 10 * powJ;
    var right = self[ind] % powJ;
    int summand = val * powJ + right;
    var left = ((j + 1) == digPow) ? 0 : (self[ind] / powJ1) * powJ1;
    self[ind] = left + summand;
    if (shouldClean) { clean(); }
  }

  public int numDigits()
  {
    if (self.Count == 0) { return 0; }
    int ind = (int)(self.Count) - 1;
    int current = self[ind];
    int best = 0;
    for (int i = 0; i < digPow; ++i)
    {
      if ((current % 10) != 0) { best = i + 1; }
      current = current / 10;
    }
    return best + digPow * (int)(self.Count - 1);
  }

  public int toInt()
  {
    if (self.Count == 0) { return 0; }
    return negative ? (-self[0]) : self[0];
  }

  public static mp gcd(in mp aa, in mp bb)
  {
    var zero_ = zero();
    mp one_ = new mp(1);
    mp two = new mp(2);
    var aa0 = new mp(aa); var bb0 = new mp(bb);
    if ((aa0 == bb0) || (bb0 == zero_)) { return (aa0 > zero_) ? aa0 : (-aa0); }
    if (aa0 == zero_) { return (bb0 > zero_) ? bb0 : (-bb0); }
    mp abs_a = (aa0 > zero_) ? aa0 : -aa0;
    mp abs_b = (bb0 > zero_) ? bb0 : -bb0;
    //if (bb0 != 0) { return gcd(bb0, aa0 % bb0); }
    for (mp safety_counter = two * abs_a + two * abs_b; bb0 != zero_; safety_counter = safety_counter - one_)
    {
      if (safety_counter <= zero_) { break; }
      mp aa0_old = aa0;
      mp bb0_old = bb0;
      aa0 = bb0_old;
      bb0 = aa0_old % bb0_old;
    }
    if (aa0 < zero_) { return -aa0; }
    return aa0;
  }

  public static mp gcd(in List<mp> arguments)
  {
    if (arguments.Count == 0) { throw new System.Exception("Cannot take gcd of no integers."); }
    int numArgs = (int)arguments.Count;
    mp answer = new mp(arguments[0]);
    for (int ii = 1; ii < numArgs; ++ii)
    {
      answer = gcd(answer, arguments[ii]);
    }
    return answer;
  }

  /** \return { a, b } where a is the maximal number such that this integer == a * a * b */
  public KeyValuePair<mp, mp> separateSquaredPart()
  {
    var answerFirst = new mp();
    var answerSecond = new mp();
    var zero_ = zero();
    if (this == zero_)
    {
      answerFirst = zero_;
      answerSecond = zero_;
      return new KeyValuePair<mp, mp>(answerFirst, answerSecond);
    }
    answerFirst = new mp(1);
    answerSecond = new mp(1);
    var factors = primeFactorization();
    foreach (var iter in factors)
    {
      if ((iter.Value % 2) == 1)
      {
        if (answerFirst < zero_)
        {
          answerSecond = answerSecond * iter.Key;
          continue;
        }
        answerFirst = answerFirst * iter.Key.pow((iter.Value - 1) / 2);
        answerSecond = answerSecond * iter.Key;
        continue;
      }
      if (answerFirst < zero_) { continue; }
      answerFirst = answerFirst * iter.Key.pow(iter.Value / 2);
    }
    return new KeyValuePair<mp, mp>(answerFirst, answerSecond);
  }


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
  Dictionary<mp, int> primeFactorization() const;

  friend std::ostream& operator<<(std::ostream& strm, const mp& mpIn);
};

std::ostream& operator<<(std::ostream& strm, const mp& mpIn);

}

#endif //def MP_INTEGER_H
