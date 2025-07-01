/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

using function_calculator_cs;
using System.Numerics;

namespace function_calculator_cs
{

/** \brief A number which is the sum of square roots of integers. */
public class QuadraticNumber
{
  /** \brief The keys represent which numbers the square roots are taken of. The values are coefficients.
   *
   * So, for example (33 / 4) + 2 * sqrt(2) + 4 * sqrt(3) - (20 / 7) * sqrt(6)
   * would be represented as:
   * content[1] = 33/ 4; content[2] = 2; content[3] = 4; content[6] = (-20 / 7);
   */
  private SortedDictionary<mp, Rational> content = new SortedDictionary<mp, Rational>();
  /** \brief From Galois Theory, multiplication acts as a linear transformation upon vector space where the square roots are basis elements.
   *  \param root2Index is an output parameter that assigns a row index to its corresponding square root.
   *  \param index2Root is an output parameter that assigns to each row index its corresponding square root.
   */
  private MatrixRational getMultiplicationMatrix(ref Dictionary<mp, int> root2Index, ref Dictionary<int, mp> index2Root)
  {
    root2Index = new Dictionary<mp, int>();
    index2Root = new Dictionary<int, mp>();
    {
      HashSet<mp> rootsSoFar = new HashSet<mp>();
      int NN = (int)content.Count;
      var current = new QuadraticNumber(this);
      for (int II = 0; II < (NN + 1); ++II)
      {
        foreach (var iter in current.content)
        {
          rootsSoFar.Add(new mp(iter.Key));
        }
        current = current * (this);
      }
      int ii = 0;
      foreach (var rootSoFar in rootsSoFar)
      {
        root2Index[rootSoFar] = ii; index2Root[ii] = rootSoFar; ++ii;
      }
    }
    int dimMatrix = (int)root2Index.Count;
    var answer = MatrixRational.zeroMatrix(dimMatrix);
    foreach (var iter in content)
    {
      MatrixRational summand = new MatrixRational();
      var radA = new mp(iter.Key);
      for (int ii = 0; ii < dimMatrix; ++ii)
      {
        List<Rational> row = new List<Rational>();
        for (int jj = 0; jj < dimMatrix; ++jj) { row.Add(new Rational()); }
        var radB = new mp(index2Root[ii]);
        var root = radA * radB;
        var sqrtSplit = root.separateSquaredPart();
        row[root2Index[sqrtSplit.Value]] = new Rational(sqrtSplit.Key, new mp(1));
        summand.addRow(row);
      }
      answer = answer + summand.transpose() * iter.Value;
    }
    return answer;
  }

  public QuadraticNumber()
  {
    content = new SortedDictionary<mp, Rational>();
  }

  public QuadraticNumber(in Rational number)
  {
    content = new SortedDictionary<mp, Rational>();
    if (number != new Rational())
    {
      var one_ = new mp(1);
      content[one_] = number;
    }
  }

  public QuadraticNumber(in QuadraticNumber other)
  {
    if (other is null) { throw new System.Exception("Trying to copy a null quadratic number."); }
    content = new SortedDictionary<mp, Rational>(other.content);
  }

  public static QuadraticNumber zero()
  {
    return new QuadraticNumber();
  }

  public virtual double toDouble()
  {
    double answer = 0.0;
    foreach (var iter in content)
    {
      if (iter.Value == Rational.zero()) { continue; }
      double val = iter.Value.toDouble();
      double radicand = (double)iter.Key.toInt();
      if (iter.Key < mp.zero()) { radicand = -radicand; throw new System.Exception("\nRadicands should be nonnegative."); }
      answer += val * Math.Sqrt(radicand);
    }
    return answer;
  }

  public Boolean isCompound() { return (content.Count > 1); }

  /** \return { a, b } where this number == a / b AND a has only integer coefficients. */
  public KeyValuePair<QuadraticNumber, mp> factorAsIntegral()
  {
    mp answerSecond = new mp(1);
    foreach (var iter in content)
    {
      answerSecond = answerSecond * iter.Value.denominator();
    }
    List<mp> numerators = new List<mp>();
    numerators.Add(answerSecond);
    QuadraticNumber answerFirst0 = new QuadraticNumber(this);
    foreach (var iter in content)
    {
      var newVal = iter.Value * (new Rational(answerSecond, new mp(1)));
      answerFirst0.content[iter.Key] = newVal;
      numerators.Add(newVal.numerator());
    }
    var gcd_ = mp.gcd(numerators);
    answerSecond = answerSecond / gcd_;
    var answerFirst = new QuadraticNumber(answerFirst0);
    foreach (var iter in answerFirst0.content)
    {
      answerFirst.content[iter.Key] = iter.Value * (new Rational(new mp(1), gcd_));
    }
    var answer = new KeyValuePair<QuadraticNumber, mp>(answerFirst, answerSecond);
    return answer;
  }

  public Boolean getRational(ref Rational self) /**< \return `true` iff the number is actually rational. Only then is self redefined. */
  {
    if (content.Count == 0) { self = new Rational(0, 1); return true; }
    if (content.Count > 1) { return false; }
    var one_ = new mp(1);
    var val = new Rational();
    Boolean answer = content.TryGetValue(one_, out val);
    if (!answer) { return false; }
    if (val is null) { return false; }
    self = new Rational(val);
    return true;
  }


  public override string ToString()
  {
    return ToString(false);
  }

  public string ToString(Boolean useParentheses)
  {
    string strm = "";
    int count = -1;
    if (useParentheses) { strm += "("; }
    if (content.Count == 0) { strm += "0"; }
    foreach (var iter in content)
    {
      var val = new Rational(iter.Value);
      if (val == Rational.zero()) { continue; }
      ++count;
      if (count > 0)
      {
        if (val >= Rational.zero()) { strm += " + "; }
        else
        {
          val = -val;
          strm += " - ";
        }
      }
      mp one_ = new mp(1);
      Boolean coeffIsOne = (val == new Rational(one_, one_));
      var radicand = new mp(iter.Key);
      bool printCoeffParents = (val.denominator() != one_) && (radicand != one_);
      if ((radicand == one_) || (!coeffIsOne)) { strm += val.ToString(printCoeffParents); }
      if (radicand == one_) { continue; }
      Boolean complex = false;
      if (radicand < mp.zero()) { radicand = -radicand; complex = true; }
      if (!coeffIsOne) { strm += " * "; }
      if (!complex || (radicand != one_))
      {
        strm += "Sqrt(" + radicand + ")";
        if (complex) { strm += " * "; }
      }
      if (complex) { strm += "i"; }
    }
    if (useParentheses) { strm += ")"; }
    return strm;
  }

  public static QuadraticNumber sqrt(int radicand)
  {
    Rational rad = new Rational(radicand, 1);
    return sqrt(rad);
  }

  public static QuadraticNumber sqrt(in mp radicand)
  {
    Rational rad = new Rational(radicand, new mp(1));
    return sqrt(rad);
  }

  public static QuadraticNumber sqrt(in Rational radicand)
  {
    QuadraticNumber answer = new QuadraticNumber();
    var zero_ = Rational.zero();
    var one_ = new mp(1);
    var minusOne = new mp(-1);
    if (radicand == zero_) { return answer; }
    if (radicand < zero_) { throw new System.Exception("Radicand should be nonnegative."); }
    Rational coefficient = new Rational(one_, radicand.denominator());
    mp key = radicand.numerator() * radicand.denominator();
    var primes = key.primeFactorization();
    foreach (var iter in primes)
    {
      var factor = new mp(iter.Key);
      if (factor == minusOne) { continue; }
      var power = iter.Value;
      if (power <= 1) { continue; }
      int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
      Rational sqrtRational = (new Rational(factor, one_)).pow(coeffPow);
      coefficient = coefficient * sqrtRational;
      key = key / (sqrtRational * sqrtRational).numerator();
    }
    answer.content[key] = coefficient;
    return answer;
  }

  public QuadraticNumber abs()
  {
    return ((this) < zero()) ? (-(this)) : (+(this));
  }

  public static QuadraticNumber operator+(in QuadraticNumber body) { return new QuadraticNumber(body); }

  public static QuadraticNumber operator-(in QuadraticNumber body)
  {
    var answer = new QuadraticNumber();
    foreach (var iter in body.content) { answer.content[iter.Key] = -iter.Value; }
    return answer;
  }

  public static QuadraticNumber operator+(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    HashSet<mp> added = new HashSet<mp>();
    QuadraticNumber sum = new QuadraticNumber();
    mp one_ = new mp(1);
    mp minusOne = new mp(-1);
    foreach (var iter in body.content)
    {
      mp radicand = new mp(iter.Key);
      Rational coeff = new Rational(iter.Value);
      Dictionary<mp, int> primes = radicand.primeFactorization();
      foreach (var jter in primes)
      {
          var factor = jter.Key;
          if (factor == minusOne) { continue; }
          var power = jter.Value;
          if (power <= 1) { continue; }
          int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
          Rational sqrtRational = (new Rational(factor, one_)).pow(coeffPow);
          coeff = coeff * sqrtRational;
          radicand = radicand / (sqrtRational * sqrtRational).numerator();
      }
      Rational? summand = new Rational();
      Boolean found = rhs.content.TryGetValue(radicand, out summand);
      if ((!found) || (summand is null))
      {
        sum.content[radicand] = coeff;
      }
      else if (summand != (-coeff))
      {
        sum.content[radicand] = summand + coeff;
      }
      added.Add(radicand);
    }
    foreach (var iter in rhs.content)
    {
      mp radicand = new mp(iter.Key);
      Rational coeff = new Rational(iter.Value);
      Dictionary<mp, int> primes = radicand.primeFactorization();
      foreach (var jter in primes)
      {
        var factor = jter.Key;
        if (factor == minusOne) { continue; }
        var power = jter.Value;
        if (power <= 1) { continue; }
        int coeffPow = (power % 2 == 0) ? (power / 2) : ((power - 1) / 2);
        Rational sqrtRational = (new Rational(factor, one_)).pow(coeffPow);
        coeff = coeff * sqrtRational;
        radicand = radicand / (sqrtRational * sqrtRational).numerator();
      }
      if (added.Contains(radicand)) { continue; }
      sum.content[radicand] = coeff;
    }
    return sum;
  }

  public static QuadraticNumber operator-(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    return body + (-rhs);
  }

  public static QuadraticNumber operator*(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    QuadraticNumber product = new QuadraticNumber();
    var one_ = new mp(1);
    foreach (var iter in body.content)
    {
      foreach (var jter in rhs.content)
      {
        var summand0 = sqrt((new Rational(iter.Key, one_)) * (new Rational(jter.Key, one_)));
        var factor = iter.Value * jter.Value;
        QuadraticNumber summand = new QuadraticNumber();
        foreach (var kter in summand0.content)
        {
          summand.content[kter.Key] = kter.Value * factor;
        }
        product = product + summand;
      }
    }
    return product;
  }

  /** \brief Uses inversion of the multiplication operator. */
  public static QuadraticNumber operator/(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    if (body.content.Count == 0) { return new QuadraticNumber(body); }
    if (rhs == zero()) { throw new System.Exception("Division by zero."); }
    if (rhs.content.Count == 1)
    {
      var multiplicand = new QuadraticNumber();
      Rational one_ = new Rational(1);
      foreach (var iter in rhs.content)
      {
        Rational radicand = new Rational(iter.Key, new mp(1));
        multiplicand.content[iter.Key] = one_ / (radicand * iter.Value);
      }
      return body * multiplicand;
    }
    Dictionary<int, mp> index2Root = new Dictionary<int, mp>();
    Dictionary<mp, int> root2Index = new Dictionary<mp, int>();
    MatrixRational multMatrix = rhs.getMultiplicationMatrix(ref root2Index, ref index2Root);
    Boolean success = false;
    MatrixRational multInverse = multMatrix.inverse(ref success);
    if (!success) { throw new System.Exception("Division failed."); }
    var dim = multMatrix.numRows();
    MatrixRational multVector = new MatrixRational();
    {
      List<Rational> row = new List<Rational>();
      for (int ii = 0; ii < dim; ++ii) { row.Add(Rational.zero()); }
      row[root2Index[new mp(1)]] = new Rational(1); // root2Index guaranteed to have 1 as a key since sqrt(A)^2 = A * sqrt(1)
      multVector.addRow(row);
      multVector = multVector.transpose();
    }
    multVector = multInverse * multVector;
    QuadraticNumber reciprocal = new QuadraticNumber();
    for (int ii = 0; ii < dim; ++ii)
    {
      Rational coeff = multVector.at(ii, 0);
      if (coeff == Rational.zero()) { continue; }
      reciprocal.content[index2Root[ii]] = coeff;
      //reciprocal = reciprocal + QuadraticNumber::sqrt(Rational(index2Root[ii], 1)) * multVector.at(ii, 0);
    }
    return body * reciprocal;
  }

  public QuadraticNumber pow(int p) /**< `return` The p'th power of the number. */
  {
    Boolean isNeg = (p < 0);
    if (isNeg) { p = -p; }
    Rational one_ = new Rational(1);
    QuadraticNumber answer = new QuadraticNumber(one_);
    for (int ii = 0; ii < p; ++ii)
    {
      answer = answer * (this);
    }
    if (isNeg)
    {
      return new QuadraticNumber(one_) / answer;
    }
    return answer;
  }

  public QuadraticNumber powerOf(in mp p) /**< `return` The p'th power of the number. */
  {
    var oneMp = new mp(1);
    var one_ = new QuadraticNumber(new Rational(1));
    if (p == mp.zero()) { return one_; }
    Boolean isNeg = (p < mp.zero());
    if (isNeg) { return one_ / powerOf(-p); }
    var factors = p.primeFactorization();
    var answer = new QuadraticNumber(one_);
    if (factors.Count <= 1)
    {
      for (mp i = mp.zero(); i < p; i = i + oneMp)
      {
        answer = answer * (this);
      }
      return answer;
    }
    QuadraticNumber baseNum = new QuadraticNumber(this);
    while (factors.Count > 0)
    {
      answer = new QuadraticNumber(one_);
      var powerPair = factors.FirstOrDefault();
      var power_ = powerPair.Key.pow(powerPair.Value);
      var prevCount = factors.Count;
      factors.Remove(powerPair.Key);
      if (factors.Count >= prevCount) { break; }
      for (mp i = mp.zero(); i < power_; i = i + oneMp)
      {
        answer = answer * baseNum;
      }
      baseNum = new QuadraticNumber(answer);
    }
    return answer;
  }

  public override bool Equals(object? obj)
  {
    return Equals(obj as QuadraticNumber);
  }

  public static Boolean operator==(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    return body.Equals(rhs);
  }

  public Boolean Equals(QuadraticNumber? rhs)
  {
    if (rhs is null) { return false; }
    foreach (var iter in content)
    {
      Rational? val = new Rational();
      Boolean found = rhs.content.TryGetValue(iter.Key, out val);
      if (!found) { return false; }
      if (val is null) { return false; }
      if (val != iter.Value) { return false; }
    }
    foreach (var iter in rhs.content)
    {
      Rational? val = new Rational();
      Boolean found = content.TryGetValue(iter.Key, out val);
      if (!found) { return false; }
      if (val is null) { return false; }
      if (val != iter.Value) { return false; }
    }
    return true;
  }

  public override int GetHashCode()
  {
    var hash = new HashCode();
    foreach (var iter in content) { hash.Add(iter); }
    return hash.ToHashCode();
  }

  public static Boolean operator!=(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    return !(body == rhs);
  }

  public static Boolean operator==(in QuadraticNumber body, int rhs)
  {
    return (body == (new QuadraticNumber(new Rational(rhs))));
  }

  public static Boolean operator!=(in QuadraticNumber body, int rhs)
  {
    return !(body == rhs);
  }

  /** \remark Does not use algebra to determine < since it would be very inefficient. */
  public static Boolean operator<(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    return (body.toDouble() < rhs.toDouble());
  }

  public static Boolean operator>(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    return (rhs < body);
  }

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  //public static Boolean tryGetCosine(const Rational& input, QuadraticNumber& output);

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  //public static Boolean tryGetSine(const Rational& input, QuadraticNumber& output);
}
}

