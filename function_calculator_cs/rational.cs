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

  public static Boolean operator!=(in Rational body, in Rational rhs) const;
  public static Boolean operator<(in Rational body, in Rational rhs) const;
  public static Boolean operator>(in Rational body, in Rational rhs) const;
  public static Boolean operator<=(in Rational body, in Rational rhs) const;
  public static Boolean operator>=(in Rational body, in Rational rhs) const;
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  public std::map<mp, int> primeFactorization() const;
  /** \brief Print the prime factorization of the rational number. */
  public std::string printFactors(bool useParentheses = false) const;

  public virtual std::pair<double, double> get() const;
  public Boolean isInt() const;
  public virtual std::string print(bool useParentheses = false) const override;

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

