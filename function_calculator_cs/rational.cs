/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

namespace function_calculator_cs
{

public class Rational
{
  private mp num = new mp(0);
  private mp denom = new mp(1);

  static Rational()
  {
  }

  public Rational(int nn = 0, int dd = 1) : this(new mp(nn), new mp(dd))
  {
  }

  public Rational(in mp nn, in mp dd)
  {
    var zero_ = mp.zero();
    if (dd == zero_)
    {
      throw new System.Exception("Division by zero.");
    }
    else if (nn == zero_)
    {
      num = new mp(nn);
      denom = new mp(1);
    }
    else
    {
      mp gcd_ = mp.gcd(nn, dd);
      num = new mp(nn); denom = new mp(dd);
      if (gcd_ != zero_)
      {
        num = num / gcd_;
        denom = denom / gcd_;
      }
      if (denom < zero_)
      {
        num = num * (new mp(-1));
        denom = denom * (new mp(-1));
      }
    }
  }

  public Rational(in Rational rhs)
  {
    if (rhs is null) { throw new System.Exception("Copy constructor given null parameter."); }
    num = rhs.num;
    denom = rhs.denom;
  }

  public static Rational zero() { return new Rational(0, 1); }

  public mp denominator() { return new mp(denom); }
  public mp numerator() { return new mp(num); }

  /** \return { a, b } where a is the maximal number such that this number == a * a * b */
  public KeyValuePair<Rational, Rational> separateSquaredPart()
  {
    var sqPartNum = num.separateSquaredPart();
    var sqPartDen = denom.separateSquaredPart();
    KeyValuePair<Rational, Rational> answer = new KeyValuePair<Rational, Rational>(
      new Rational(sqPartNum.Key, sqPartDen.Key),
      new Rational(sqPartNum.Value, sqPartDen.Value));
    return answer;
  }

  public static Rational operator+(in Rational body) { return new Rational(body); }
  public static Rational operator-(in Rational body)
  {
    return new Rational(-body.num, body.denom);
  }

  public static Rational operator+(in Rational body, in Rational rhs)
  {
    return new Rational(body.num * rhs.denom + rhs.num * body.denom, body.denom * rhs.denom);
  }

  public static Rational operator-(in Rational body, in Rational rhs)
  {
    return (body + (-rhs));
  }

  public static Rational operator*(in Rational body, in Rational rhs)
  {
    return new Rational(body.num * rhs.num, body.denom * rhs.denom);
  }

  public static Rational operator/(in Rational body, in Rational rhs)
  {
    if (rhs.num == mp.zero())
    {
      throw new System.Exception("Operator division by zero.");
    }
    return new Rational(rhs.num * rhs.denom, rhs.denom * rhs.num);
  }

  public Rational pow(int p) /**< Returns the p'th power of the rational number. */
  {
    Boolean isNeg = (p < 0);
    if (isNeg) { p = -p; }
    Rational answer = new Rational(1, 1);
    for (int i = 0; i < p; ++i)
    {
      answer = answer * (this);
    }
    if (isNeg)
    {
      return new Rational(answer.denom, answer.num);
    }
    return answer;
  }

  public static Boolean operator==(in Rational body, in Rational rhs) { return body.Equals(rhs); }

  public static Boolean operator!=(in Rational body, in Rational rhs)
  {
    if (body == rhs) { return false; }
    return true;
  }

  public static Boolean operator<(in Rational body, in Rational rhs)
  {
    if (body.num * rhs.denom < body.denom * rhs.num) { return true; } // denom always > 0
    return false;
  }

  public static Boolean operator>(in Rational body, in Rational rhs) { return (rhs < body); }
  public static Boolean operator<=(in Rational body, in Rational rhs)
  {
    if (body == rhs) { return true; }
    return (body < rhs);
  }

  public static Boolean operator>=(in Rational body, in Rational rhs)
  {
    if (body == rhs) { return true; }
    return (body > rhs);
  }

  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  public Dictionary<mp, int> primeFactorization()
  {
    var numFactors = num.primeFactorization();
    var denomFactors = denom.primeFactorization();
    var one_ = new mp(1);
    foreach (var iter in denomFactors)
    {
      if (iter.Key == one_) { continue; }
      int iterSecond = 0;
      Boolean found = numFactors.TryGetValue(iter.Key, out iterSecond);
      if (!found)
      {
        numFactors[iter.Key] = -iter.Value;
        continue;
      }
      numFactors[iter.Key] -= iter.Value;
    }
    Dictionary<mp, int> answer = new Dictionary<mp, int>();
    int countFactors = (int)numFactors.Count;
    foreach (var iter in numFactors)
    {
      mp baseMp = new mp(iter.Key);
      int power = iter.Value;
      if (baseMp == new mp(-1))
      {
        if (power < 0) { power = -power; }
        power = (power % 2);
        if ((countFactors == 1) && (power == 0)) { baseMp = new mp(one_); power = 1; }
      }
      if ((baseMp == one_) && (countFactors > 0)) { power = 0; }
      if (power == 0)
      {
        if (countFactors == 1) { baseMp = new mp(one_); power = 1; }
        else { continue; }
      }
      answer[baseMp] = power;
    }
    return answer;
  }

  /** \brief Print the prime factorization of the rational number. */
  public string printFactors(Boolean useParentheses = false)
  {
    string strm = "";
    var zero_ = mp.zero();
    if (useParentheses) { strm += "("; }
    var factors = primeFactorization();
    int count = -1;
    foreach (var iter in factors)
    {
      if (iter.Value == 0) { continue; }
      ++count;
      if (count > 0) { strm += " * "; }
      if (iter.Key < zero_) { strm += "("; }
      strm += iter.Key;
      if (iter.Key < zero_) { strm += ")"; }
      strm += "^" + iter.Value;
    }
    if (useParentheses) { strm += ")"; }
    return strm;
  }

  public virtual double toDouble()
  {
    double nn = (double)(num.toInt());
    double dd = (double)(denom.toInt());
    return nn / dd;
  }

  public Boolean isInt()
  {
    return (denom == new mp(1));
  }

  public override string ToString()
  {
    return ToString(false);
  }

  public string ToString(Boolean useParentheses)
  {
    string strm = "";
    if (num == mp.zero()) { strm = strm + num; }
    else if (denom == new mp(1)) { strm = strm + num; }
    else { strm  = strm + num + " / " + denom; }
    if (useParentheses) { return "(" + strm + ")"; }
    return strm;
  }

  public override bool Equals(object? obj)
  {
    return Equals(obj as Rational);
  }

  public bool Equals(Rational? other)
  {
    if (other is null) { return false; }
    if (other.num != num) { return false; }
    if (other.denom != denom) { return false; }
    return true;
  }

  public override int GetHashCode()
  {
    return HashCode.Combine(num, denom);
  }
}
}

