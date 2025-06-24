/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

namespace function_calculator_cs
{

public class Rational
{
  private mp num = new mp(0);
  private mp denom = new mp(1);

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

  public Rational operator+() const;
  public Rational operator-() const;
  public Rational operator+(const Rational& rhs) const;
  public Rational operator-(const Rational& rhs) const;
  public Rational operator*(const Rational& rhs) const;
  public Rational operator/(const Rational& rhs) const;
  public Rational pow(int p) const; /**< Returns the p'th power of the rational number. */
  public bool operator==(const Rational& rhs) const;
  public bool operator!=(const Rational& rhs) const;
  public bool operator<(const Rational& rhs) const;
  public bool operator>(const Rational& rhs) const;
  public bool operator<=(const Rational& rhs) const;
  public bool operator>=(const Rational& rhs) const;
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  public std::map<mp, int> primeFactorization() const;
  /** \brief Print the prime factorization of the rational number. */
  public std::string printFactors(bool useParentheses = false) const;

  public virtual std::pair<double, double> get() const;
  public bool isInt() const;
  public virtual std::string print(bool useParentheses = false) const override;
}
}

