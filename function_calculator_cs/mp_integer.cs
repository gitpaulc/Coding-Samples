/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

#pragma warning disable CS8981 // Do not warn about lowercase class names.

namespace function_calculator_cs
{
/** \class Multiple-precision integer.
 *
 *  Important for avoiding overflow as calculations become more complicated.
 */
public class mp : Object
{
  /** \brief Stored in reverse place-value as a_0 + a_1 * b + ... + a_p * b^p. */
  private List<int> self;
  private Boolean negative = false;

  private static int digPow = 6;
  private static int intPow(int baseInt, int p)
  {
    if (p < 0) { throw new System.Exception("Exponent must be nonnegative."); }
    int answer = 1;
    for (int ii = 0; ii < p; ++ii) { answer *= baseInt; }
    return answer;
  }
  private static int limit = intPow(10, digPow);
  private struct IntPair { public int x; public int y; }
  private static Dictionary<IntPair, mp> binoms = new Dictionary<IntPair, mp>();
  private void clean()
  {
    int siz = self.Count;
    int newSiz = siz;
    for (int i = siz - 1; i >= 0; --i)
    {
      if (self[i] != 0) { break; }
      newSiz--;
    }
    if (newSiz <= 0) { self.Clear(); }
    else if (newSiz < siz)
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


  public static mp operator+(in mp body) { return new mp(body); }

  public static mp operator-(in mp body)
  {
    var answer = new mp(body);
    if (answer.self.Count == 0) { return answer; }
    answer.negative = !answer.negative;
    return answer;
  }

  public static mp operator+(in mp body, in mp rhs)
  {
    if (body.negative && rhs.negative) { return -((-rhs) + (-body)); }
    if (rhs.negative) { return (body - (-rhs)); }
    if (body.negative) { return (rhs - (-body)); }
    if (rhs.self.Count > body.self.Count) { return (rhs + body); }
    mp answer = new mp(body);
    var rhsSize = rhs.self.Count;

    int ii = -1;
    int carry = 0;
    long summandA = 0;
    long summandB = 0;
    long lim = limit;
    foreach (var iter in rhs.self)
    {
      ++ii;
      if (ii >= rhsSize) { continue; }
      summandA = answer.self[ii];
      summandB = iter;
      var sum = summandA + summandB + carry;
      carry = 0;
      if (sum >= (long)lim)
      {
        carry = 1;
        sum = sum % lim;
      }
      answer.self[ii] = (int)sum;
    }
    if (carry > 0)
    {
      answer.self.Add(carry);
    }
    answer.clean();
    return answer;
  }

  public static mp operator-(in mp body, in mp rhs)
  {
    if (rhs.self.Count == 0) { return body; }
    if (body.self.Count == 0) { return -rhs; }
    if (rhs.negative && (!body.negative)) { return body + (-rhs); }
    if (rhs.negative && body.negative) { return ((-rhs) - (-body)); }
    // rhs is nonnegative:
    if (body.negative) { return -((-body) + rhs); }
    // Both are nonnegative...

    int numOfDigits = body.numDigits();
    int numRhsDigits = rhs.numDigits();

    // Negative answer:
    if (numRhsDigits > numOfDigits) { return -(rhs - body); }
    if (numRhsDigits == numOfDigits)
    {
      for (int ii = numOfDigits - 1; ii >= 0; --ii)
      {
        int digit = rhs.getDigit(ii);
        int subFrom = body.getDigit(ii);
        if (digit > subFrom) { return -(rhs - body); }
        if (digit < subFrom) { break; }
      }
    }

    //Nonnegative number:
    mp answer = new mp(0);
    answer.self = new List<int>(body.self.Count);
    for (int ii = 0; ii < answer.self.Count; ++ii) { answer.self[ii] = 0; }
    answer.negative = false;
    var from = new mp(body);
    for (int ii = 0; ii < numOfDigits; ++ii)
    {
      int digit = rhs.getDigit(ii);
      int subFrom = from.getDigit(ii);
      if (digit > subFrom)
      {
        if (ii == (numOfDigits - 1)) { throw new System.Exception("Bad subtraction."); }
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

  public static mp operator*(in mp body, in mp rhs)
  {
    mp answer = new mp(0);
    if (body.self.Count == 0) { return answer; }
    if (rhs.self.Count == 0) { return answer; }
    answer.negative = (body.negative || rhs.negative) && !(body.negative && rhs.negative);

    int sizA = (int)body.self.Count;
    int sizB = (int)rhs.self.Count;
    {
      int maxSiz = sizA;
      if (sizB > sizA) { maxSiz = sizB; }
      int newSiz = maxSiz * maxSiz + 1;
      answer.self = new List<int>(newSiz);
      for (int ii = 0; ii < answer.self.Count; ++ii) { answer.self[ii] = 0; }
    }
    long lim = (long)limit;

    for (int ii = 0; ii < sizA; ++ii)
    {
      int carry = 0;
      for (int jj = 0; jj < sizB; ++jj)
      {
        var kk = ii + jj;
        int answerSize = answer.self.Count;
        if (kk >= answerSize)
        {
          answer.self.AddRange(new List<int>(kk + 1 - answerSize));
          for (int ll = answerSize; ll < kk + 1; ++ll) { answer.self[ll] = 0; }
        }
        long product = (long)(body.self[ii]) * (long)(rhs.self[jj]) + carry;
        carry = (int)(product / lim);
        answer.self[kk] = answer.self[kk] + ((int)(product % lim));
      }
      if (carry > 0)
      {
        var kk = ii + sizB;
        int answerSize = answer.self.Count;
        if (kk >= answerSize)
        {
          answer.self.AddRange(new List<int>(kk + 1 - answerSize));
          for (int ll = answerSize; ll < kk + 1; ++ll) { answer.self[ll] = 0; }
        }
        answer.self[kk] = answer.self[kk] + carry;
      }
    }
    answer.clean();
    return answer;
  }

  public static mp operator*(in mp body, int rhs) { var rhs_ = new mp(rhs); return body * rhs_; }

  public static mp operator/(in mp body, in mp rhs)
  {
    mp remainder = new mp(0);
    var quotient = body.division(rhs, ref remainder);
    return quotient;
  }

  public static mp operator%(in mp body, in mp rhs)
  {
    mp remainder = new mp(0);
    var quotient = body.division(rhs, ref remainder);
    return remainder;
  }

  mp abs()
  {
    mp answer = new mp(this);
    answer.negative = false;
    return answer;
  }

  mp pow(int p) /**< `return` The p'th power of the number. */
  {
    Boolean isNeg = (p < 0);
    if (isNeg) { throw new System.Exception("Exponent must be nonnegative."); }
    mp answer = new mp(1);
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (this);
    }
    return answer;
  }

  public static bool operator==(in mp body, in mp rhs)
  {
    var diff = body - rhs;
    foreach (var iter in diff.self)
    {
      if (iter != 0) { return false; }
    }
    return true;
  }

  public static bool operator!=(in mp body, in mp rhs) { return !(body == rhs); }

  public static bool operator<(in mp body, in mp rhs)
  {
    var diff = body - rhs;
    return diff.negative;
  }

  public static bool operator>(in mp body, in mp rhs) { return (rhs < body); }

  public static bool operator<=(in mp body, in mp rhs)
  {
    if (body == rhs) { return true; }
    return (body < rhs);
  }

  public static bool operator>=(in mp body, in mp rhs) { return (rhs <= body); }

  /** \return n! / ((n - k)! * k!) */
  public static mp binomialCoeff(int n, int k)
  {
    if (n < 0) { return zero(); }
    if (k < 0) { return zero(); }
    if (k > n) { return zero(); }
    if (binoms.Count == 0)
    {
      var zeroZero = new IntPair();
      zeroZero.x = 0;
      zeroZero.y = 0;
      binoms[zeroZero] = new mp(1);
    }
    var nK = new IntPair();
    nK.x = n;
    nK.y = k;
    var iterSecond = new mp(0);
    Boolean found = binoms.TryGetValue(nK, out iterSecond);
    if (found && !(iterSecond is null)) { return iterSecond; }
    var answer = binomialCoeff(n - 1, k) + binomialCoeff(n - 1, k - 1);
    binoms[nK] = answer;
    return answer;
  }

  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  public Dictionary<mp, int> primeFactorization()
  {
    var input = new mp(this);
    var one_ = new mp(1);
    var two_ = new mp(1);
    var answer = new Dictionary<mp, int>();
    if (input * input <= new mp(1))
    {
      answer[input] = 1;
      return answer;
    }
    var zero_ = zero();
    if (input < zero_) { answer[new mp(-1)] = 1; input = -input; }
    mp lim = input + one_;
    HashSet<mp> sieved = new HashSet<mp>();
    bool foundFactor = false;
    for (mp init = new mp(two_); init < lim; init = init + one_)
    {
      for (mp factor = new mp(init); factor < lim; factor = factor + init)
      {
        if (sieved.Contains(factor)) { continue; }
        sieved.Add(factor);
        if (input % factor != zero_) { continue; }
        foundFactor = true;
        if (factor == input)
        {
          if (!(answer.ContainsKey(factor)))
          {
            answer[factor] = 1;
            break;
          }
          answer[factor] = answer[factor] + 1;
          break;
        }
        if (!(answer.ContainsKey(factor)))
        {
          answer[factor] = 1;
        }
        else { answer[factor] = answer[factor] + 1; }
        var quotient = input / factor;
        if (quotient.abs() >= input.abs()) // Should never happen.
        {
          throw new System.Exception("Bad prime factorization."); // break;
        }
        var others = quotient.primeFactorization();
        foreach (var iter in others)
        {
          if (!(answer.ContainsKey(iter.Key)))
          {
            answer[iter.Key] = iter.Value;
            continue;
          }
          answer[iter.Key] += iter.Value;
        }
        break;
      }
      if (foundFactor) { break; }
    }
    return answer;
  }

  public override string ToString()
  {
    var zero_ = zero();
    if (this == zero_) { return new string("0"); }
    if (negative) { return new string("-") + (-(this)).ToString(); }
    string reversed = new string("");
    int digitCount = 0;
    int nn = (int)(self.Count);
    for (int ii = 0; ii < nn; ++ii)
    {
      var element = self[ii];
      for (int jj = 0; jj < digPow; ++jj)
      {
        reversed += (element % 10);
        ++digitCount;
        element = element / 10;
        if ((element == 0) && (ii == nn - 1)) { break; }
        if ((digitCount % 3) == 0) { reversed += ","; }
      }
    }
    nn = (int)reversed.Length;
    string strm = new string("");
    for (int ii = 0; ii < nn; ++ii)
    {
      strm = strm + reversed[nn - ii - 1];
    }
    return strm;
  }

  public override bool Equals(object? o) /**< Implement to remove warning. */
  {  
    return base.Equals(o);  
  }

  public override int GetHashCode() { return base.GetHashCode(); } /**< Implement to remove warning. */
};

}

