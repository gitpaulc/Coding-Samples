/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

using function_calculator_cs;
using System.Numerics;

namespace function_calculator_cs
{

/** \brief A number which is the sum of square roots of integers. */
public class QuadraticNumber : IComparable<QuadraticNumber>, IFieldElement<QuadraticNumber>
{
  static private Dictionary<QuadraticNumber, QuadraticNumber> divisionResults;
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

  private struct TableEntry
  {
    public uint numIters = 0;
    public Rational lower;
    public Rational upper;
    public TableEntry()
    {
      numIters = 0;
      lower = new Rational();
      upper = new Rational();
    }
  };
  private static Dictionary<Rational, TableEntry> cachedBounds;

  /** \brief Outputs `lower` and `upper` approximations to `radicand` after `numIterations` of bisection.
   *
   *  \remark The values are equal if and only if the radicand is rational.
   *  \remark The bisection method is guaranteed to converge, unlike Newton's method.
   *  \throw Throws an exception if the radicand is negative.
   *  \remark Accuracy may exceed that given by number of iterations.
   */
  private static void getLowerUpperBounds(in Rational radicand, in uint numIterations, ref Rational lower, ref Rational upper)
  {
    if (radicand < Rational.zero())
    {
      throw new System.Exception("Radicand must be nonnegative.");
    }
    TableEntry entry = new TableEntry();
    {
      bool found = cachedBounds.TryGetValue(radicand, out entry);
      if (found)
      {
        if (entry.lower == entry.upper)
        {
          if (entry.numIters < numIterations)
          {
            entry.numIters = numIterations;
          }
          lower = new Rational(entry.lower);
          upper = new Rational(entry.upper);
          return;
        }
        if (entry.numIters >= numIterations)
        {
          lower = new Rational(entry.lower);
          upper = new Rational(entry.upper);
          return;
        }
      }
    }
    Rational one_ = new Rational(1);
    if (radicand == one_)
    {
      lower = new Rational(one_);
      upper = new Rational(one_);
      return;
    }
    Rational lower_ = new Rational();
    Rational upper_ = new Rational();
    if (radicand > one_)
    {
      lower_ = new Rational(one_);
      upper_ = new Rational(radicand);
    }
    else // if (radicand < one_)
    {
      lower_ = new Rational(radicand);
      upper_ = new Rational(one_);
    }
    for (uint nn = 1; nn < numIterations; ++nn)
    {
      var bisection = (lower_ + upper_) * new Rational(1, 2);
      var comparer = bisection * bisection;
      if (comparer == radicand)
      {
        lower = new Rational(bisection);
        upper = new Rational(bisection);
        entry.numIters = numIterations;
        entry.lower = new Rational(lower);
        entry.upper = new Rational(upper);
        cachedBounds[radicand] = entry;
        return;
      }
      if (comparer < radicand)
      {
        lower_ = new Rational(bisection);
        continue;
      }
      // if (comparer > radicand)
      {
        upper_ = new Rational(bisection);
        continue;
      }
    }
    lower = new Rational(lower_);
    upper = new Rational(upper_);
    entry.numIters = numIterations;
    entry.lower = new Rational(lower);
    entry.upper = new Rational(upper);
    cachedBounds[radicand] = entry;
  }

  static QuadraticNumber()
  {
    divisionResults = new Dictionary<QuadraticNumber, QuadraticNumber>();
    cachedBounds = new Dictionary<Rational, TableEntry>();
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
    {
      QuadraticNumber? quotient = new QuadraticNumber();
      Boolean foundQuotient = divisionResults.TryGetValue(rhs, out quotient);
      if (foundQuotient && !(quotient is null))
      {
        return body * quotient;
      }
    }
    if (rhs.content.Count == 1)
    {
      var multiplicand = new QuadraticNumber();
      Rational one_ = new Rational(1);
      foreach (var iter in rhs.content)
      {
        Rational radicand = new Rational(iter.Key, new mp(1));
        multiplicand.content[iter.Key] = one_ / (radicand * iter.Value);
      }
      divisionResults[rhs] = multiplicand;
      divisionResults[multiplicand] = rhs;
      return body * multiplicand;
    }
    if (rhs.content.Count == 2)
    {
      mp aa = new mp(1);  mp bb = new mp(1);
      Rational cc = new Rational();  Rational dd = new Rational();
      int ii = 0;
      foreach (var iter in rhs.content)
      {
        if (ii >= 2) { break; }
        if (ii == 0) { aa = new mp(iter.Key); cc = new Rational(iter.Value); }
        else { bb = new mp(iter.Key); dd = new Rational(iter.Value); }
        ++ii;
      }
      mp one_ = new mp(1);
      Rational c_ = new Rational(cc.numerator() * dd.denominator(), one_);
      Rational d_ = new Rational(dd.numerator() * cc.denominator(), one_);
      Rational norm = (new Rational(aa, one_)) * c_ * c_ - (new Rational(bb, one_)) * d_ * d_;
      if (norm != Rational.zero())
      {
        var multiplicand = new QuadraticNumber();
        Rational factor = new Rational(cc.denominator() * dd.denominator(), one_);
        multiplicand.content[aa] = (c_ / norm) * factor;
        multiplicand.content[bb] = -(d_ / norm) * factor;
        divisionResults[rhs] = multiplicand;
        divisionResults[multiplicand] = rhs;
        return body * multiplicand;
      }
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
    divisionResults[rhs] = reciprocal;
    divisionResults[reciprocal] = rhs;
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

  /** \brief Outputs `lower` and `upper` approximations to the number after `numIterations` of bisection.
   *
   *  \remark The values are equal if and only if the number is rational.
   *  \remark The bisection method is guaranteed to converge, unlike Newton's method.
   *  \throw Throws an exception if any radicand is negative.
   *  \remark Accuracy may exceed that given by number of iterations.
   */
  public void getLowerUpperBounds(in uint numIterations, ref Rational lower, ref Rational upper)
  {
    Rational lower_ = new Rational();
    Rational upper_ = new Rational();
    Rational ll = new Rational();
    Rational uu = new Rational();
    mp one_ = new mp(1);
    foreach (var iter in content)
    {
      getLowerUpperBounds(new Rational(iter.Key, one_), numIterations, ref ll, ref uu);
      ll = ll * iter.Value;
      uu = uu * iter.Value;
      if (ll <= uu)
      {
        lower_ = lower_ + ll;
        upper_ = upper_ + uu;
        continue;
      }
      lower_ = lower_ + uu;
      upper_ = upper_ + ll;
    }
    lower = new Rational(lower_);
    upper = new Rational(upper_);
  }

  public static Boolean operator<(in QuadraticNumber body, in QuadraticNumber rhs)
  {
    if (body == rhs) { return false; }
    Rational l0 = new Rational();
    Rational u0 = new Rational();
    Rational l1 = new Rational();
    Rational u1 = new Rational();
    for (uint nn = 5; true; nn += 5)
    {
      body.getLowerUpperBounds(nn, ref l0, ref u0);
      rhs.getLowerUpperBounds(nn, ref l1, ref u1);
      if (u0 < l1) { return true; }
      if (u1 < l0) { return false; }
    }
    return false;
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

  // IComparable<QuadraticNumber>
  public int CompareTo(QuadraticNumber? other)
  {
    if (other is null) return 1;
    if (this == other) return 0;
    return (this < other) ? -1 : 1;
  }

  // IFieldElement<QuadraticNumber>
  public QuadraticNumber Add(QuadraticNumber rhs) => this + rhs;
  public QuadraticNumber Sub(QuadraticNumber rhs) => this - rhs;
  public QuadraticNumber Mul(QuadraticNumber rhs) => this * rhs;
  public QuadraticNumber Div(QuadraticNumber rhs) => this / rhs;
  public QuadraticNumber Neg() => -(this);
  public bool IsZero()
  {
    var zero_ = Rational.zero();
    foreach (var iter in content) { if (iter.Value != zero_) return false; }
    return true;
  }
  public bool FieldEquals(QuadraticNumber other) => this == other;
  public int FieldCompareTo(QuadraticNumber other) => CompareTo(other);
  public string Print(bool useParentheses = false) => ToString(useParentheses);
  public static QuadraticNumber FieldZero() => zero();
  public static QuadraticNumber FieldOne() => new QuadraticNumber(new Rational(1));

  public int getNumRootsInSum() => content.Count;
  public SortedSet<mp> getSummandRoots()
  {
    var s = new SortedSet<mp>();
    foreach (var k in content.Keys) { s.Add(new mp(k)); }
    return s;
  }

  private void clean()
  {
    var zero_ = Rational.zero();
    var mpZero = mp.zero();
    var toRemove = new List<mp>();
    foreach (var iter in content)
    {
      if (iter.Key == mpZero || iter.Value == zero_) { toRemove.Add(new mp(iter.Key)); }
    }
    foreach (var k in toRemove) { content.Remove(k); }
  }

  private QuadraticNumber coeffsAbs()
  {
    var answer = new QuadraticNumber(this);
    var zero_ = Rational.zero();
    var updates = new List<KeyValuePair<mp, Rational>>();
    foreach (var iter in answer.content)
    {
      if (iter.Value < zero_) { updates.Add(new KeyValuePair<mp, Rational>(new mp(iter.Key), -iter.Value)); }
    }
    foreach (var kv in updates) { answer.content[kv.Key] = kv.Value; }
    return answer;
  }

  private static Dictionary<QuadraticNumber, HashSet<QuadraticNumber>> sIteratesCache = new();

  private HashSet<QuadraticNumber> getIterates0()
  {
    var answer = new HashSet<QuadraticNumber>();
    if (content.Count == 0) { return answer; }
    {
      if (sIteratesCache.TryGetValue(this, out var cached)) { return cached; }
    }
    var firstEntry = content.First();
    if (!firstEntry.Value.isInt())
    {
      throw new System.Exception("Use integral quadratic number for intermediate prime factorization.");
    }
    var lim = firstEntry.Value.numerator();
    bool isNeg = (lim < mp.zero());
    var absLim = isNeg ? -lim : new mp(lim);
    var mpZero = mp.zero();
    var mpOne = new mp(1);

    if (content.Count == 1)
    {
      for (var ind = mpZero; ind <= absLim; ind = ind + mpOne)
      {
        var quad = new QuadraticNumber();
        var coeff = isNeg ? -ind : new mp(ind);
        quad.content[firstEntry.Key] = new Rational(coeff, mpOne);
        answer.Add(quad);
      }
      sIteratesCache[this] = answer;
      return answer;
    }

    var other = new QuadraticNumber();
    foreach (var jter in content)
    {
      if (jter.Key == firstEntry.Key) { continue; }
      other.content[new mp(jter.Key)] = jter.Value;
    }
    var smaller = other.getIterates0();

    for (var ind = mpZero; ind <= absLim; ind = ind + mpOne)
    {
      foreach (var remaining in smaller)
      {
        var quad = new QuadraticNumber();
        var coeff = isNeg ? -ind : new mp(ind);
        quad.content[firstEntry.Key] = new Rational(coeff, mpOne);
        foreach (var jter in remaining.content) { quad.content[new mp(jter.Key)] = jter.Value; }
        answer.Add(quad);
      }
    }
    sIteratesCache[this] = answer;
    return answer;
  }

  private HashSet<QuadraticNumber> getIterates()
  {
    var iterates0 = getIterates0();
    var iterates = new HashSet<QuadraticNumber>();
    foreach (var iterate0 in iterates0)
    {
      var iterate = new QuadraticNumber(iterate0);
      iterate.clean();
      if (iterate.IsZero()) { continue; }
      Rational rationalVal = new Rational();
      if (iterate.getRational(ref rationalVal)) { continue; }
      {
        var coeffs = new List<mp>();
        foreach (var iter in iterate.content) { coeffs.Add(iter.Value.numerator()); }
        if (coeffs.Count > 0)
        {
          var g = mp.gcd(coeffs);
          if (g != new mp(1) && g != new mp(-1)) { continue; }
        }
      }
      iterates.Add(iterate);
    }
    return iterates;
  }

  private Dictionary<QuadraticNumber, int> primeFacIntegral()
  {
    var input = new QuadraticNumber(this);
    var answer = new Dictionary<QuadraticNumber, int>();
    var lim = input.coeffsAbs();
    var sieved = new HashSet<QuadraticNumber>();
    var iterates = input.getIterates();
    bool foundFactor = false;

    foreach (var init in iterates)
    {
      if (foundFactor) { break; }
      for (var factor = new QuadraticNumber(init); factor.coeffsAbs() < lim; factor = factor + init)
      {
        if (sieved.Contains(factor)) { continue; }
        sieved.Add(new QuadraticNumber(factor));
        var quotient = input / factor;
        if (!iterates.Contains(quotient)) { continue; }
        if (factor == input) { continue; }
        foundFactor = true;
        if (!answer.ContainsKey(factor)) { answer[factor] = 1; }
        else { answer[factor]++; }
        var others = quotient.primeFacIntegral();
        foreach (var iter in others)
        {
          if (!answer.ContainsKey(iter.Key)) { answer[iter.Key] = iter.Value; }
          else { answer[iter.Key] += iter.Value; }
        }
        break;
      }
    }
    if (!foundFactor) { answer[input] = 1; }
    return answer;
  }

  public Dictionary<QuadraticNumber, int> primeFactorization()
  {
    var input = new QuadraticNumber(this);
    var answer = new Dictionary<QuadraticNumber, int>();
    {
      Rational inputAsRational = new Rational();
      if (input.getRational(ref inputAsRational))
      {
        var factors = inputAsRational.primeFactorization();
        foreach (var iter in factors)
        {
          answer[new QuadraticNumber(new Rational(new mp(iter.Key), new mp(1)))] = iter.Value;
        }
        return answer;
      }
    }
    if (input < zero())
    {
      answer[new QuadraticNumber(new Rational(new mp(-1), new mp(1)))] = 1;
      input = -input;
    }
    {
      var asIntegral = input.factorAsIntegral();
      input = asIntegral.Key;
      var factors = new Rational(new mp(1), asIntegral.Value).primeFactorization();
      foreach (var iter in factors)
      {
        if (iter.Key == new mp(1)) { continue; }
        answer[new QuadraticNumber(new Rational(new mp(iter.Key), new mp(1)))] = iter.Value;
      }
    }
    var fac = input.primeFacIntegral();
    foreach (var iter in fac)
    {
      if (!answer.ContainsKey(iter.Key)) { answer[iter.Key] = iter.Value; }
      else { answer[iter.Key] += iter.Value; }
    }
    return answer;
  }

  public KeyValuePair<QuadraticNumber, QuadraticNumber> separateSquaredPart()
  {
    if (this == zero()) { return new KeyValuePair<QuadraticNumber, QuadraticNumber>(zero(), zero()); }
    var a = new QuadraticNumber(new Rational(1));
    var b = new QuadraticNumber(new Rational(1));
    var factors = primeFactorization();
    foreach (var iter in factors)
    {
      var prim = new QuadraticNumber(iter.Key);
      int expon = iter.Value;
      if (expon < 0)
      {
        prim = new QuadraticNumber(new Rational(1)) / prim;
        expon = -expon;
      }
      if ((expon % 2) == 1)
      {
        a = a * prim.pow((expon - 1) / 2);
        b = b * prim;
        continue;
      }
      a = a * prim.pow(expon / 2);
    }
    return new KeyValuePair<QuadraticNumber, QuadraticNumber>(a, b);
  }

  public bool simpleSquareRoot(out QuadraticNumber quadSqrt)
  {
    quadSqrt = zero();
    if (content.Count > 2) { return false; }
    var one_ = new mp(1);
    if (!content.TryGetValue(one_, out var uu) || uu is null) { return false; }
    if (content.Count == 1)
    {
      if (uu < Rational.zero()) { return false; }
      quadSqrt = sqrt(uu);
      return true;
    }
    Rational vv = new Rational();
    mp dd_key = new mp(1);
    foreach (var jter in content)
    {
      if (jter.Key == one_) { continue; }
      vv = jter.Value;
      dd_key = new mp(jter.Key);
      break;
    }
    var dd = new Rational(new mp(dd_key), new mp(1));
    var two = new Rational(2);
    var radicand = uu * uu - vv * vv * dd;
    if (radicand < Rational.zero()) { return false; }
    var rad = sqrt(radicand);
    Rational radical = new Rational();
    if (!rad.getRational(ref radical)) { return false; }
    var a2 = (uu + radical) / two;
    if (a2 < Rational.zero()) { return false; }
    var aa = sqrt(a2);
    var bb = new QuadraticNumber(vv) / (aa + aa);
    var sqrtDD = sqrt(dd);
    quadSqrt = aa + bb * sqrtDD;
    return true;
  }
}
}

