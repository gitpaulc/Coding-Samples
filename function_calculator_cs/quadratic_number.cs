/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

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
  private Dictionary<mp, Rational> content = new Dictionary<mp, Rational>();
  /** \brief From Galois Theory, multiplication acts as a linear transformation upon vector space where the square roots are basis elements.
   *  \param root2Index is an output parameter that assigns a row index to its corresponding square root.
   *  \param index2Root is an output parameter that assigns to each row index its corresponding square root.
   */
  private MatrixRational getMultiplicationMatrix(ref Dictionary<mp, int> root2Index, ref Dictionary<int, mp> index2Root)
  {
  }

  public QuadraticNumber()
  {
    content = new Dictionary<mp, Rational>();
  }

  public QuadraticNumber(in Rational number)
  {
    content = new Dictionary<mp, Rational>();
    if (number != new Rational())
    {
      var one_ = new mp(1);
      content[one_] = number;
    }
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

  /** \return { a, b } where this number == a / b AND a has only integer coefficients. */
  public KeyValuePair<QuadraticNumber, mp> factorAsIntegral()
  {
    std::pair<QuadraticNumber, mp> answer;
    answer.first = *this;
    answer.second = mp(1);
    for (const auto& iter : answer.first.content)
    {
      answer.second = answer.second * iter.second.denominator();
    }
    std::vector<mp> numerators;
    numerators.push_back(answer.second);
    for (auto& iter : answer.first.content)
    {
      iter.second = iter.second * Rational(answer.second, mp(1));
      numerators.push_back(iter.second.numerator());
    }
    auto gcd_ = mp::gcd(numerators);
    answer.second = answer.second / gcd_;
    for (auto& iter : answer.first.content)
    {
      iter.second = iter.second * Rational(mp(1), gcd_);
    }
    return answer;
  }

  public Boolean getRational(Rational& self) const; /**< \return `true` iff the number is actually rational. Only then is self redefined. */

  public override string ToString()
  {
    return ToString(false);
  }

  public string ToString(Boolean useParentheses)
  {
    std::stringstream strm;
    int count = -1;
    if (useParentheses) { strm << "("; }
    if (content.size() == 0) { strm << "0"; }
    for (const auto& iter : content)
    {
      auto val = iter.second;
      if (val == 0) { continue; }
      ++count;
      if (count > 0)
      {
        if (val >= 0) { strm << " + "; }
        else
        {
          val = -val;
          strm << " - ";
        }
      }
      bool coeffIsOne = (val == 1);
      auto radicand = iter.first;
      bool printCoeffParents = (val.denominator() != 1) && (radicand != 1);
      if ((radicand == 1) || (!coeffIsOne)) { strm << val.print(printCoeffParents); }
      if (radicand == 1) { continue; }
      bool complex = false;
      if (radicand < 0) { radicand = -radicand; complex = true; }
      if (!coeffIsOne) { strm << " * "; }
      if (!complex || (radicand != 1))
      {
        strm << "Sqrt(" << radicand << ")";
        if (complex) { strm << " * "; }
      }
      if (complex) { strm << "i"; }
    }
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  public static QuadraticNumber sqrt(const Rational& radicand);
  public QuadraticNumber abs() const;

  public QuadraticNumber operator+() const;
  public static QuadraticNumber operator-() const;
  public static QuadraticNumber operator+(const QuadraticNumber& rhs) const;
  public static QuadraticNumber operator-(const QuadraticNumber& rhs) const;
  public static QuadraticNumber operator*(const QuadraticNumber& rhs) const;
  /** \brief Uses inversion of the multiplication operator. */
  public static QuadraticNumber operator/(const QuadraticNumber& rhs) const;
  public QuadraticNumber pow(int p); /**< `return` The p'th power of the number. */

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
    for (const auto& iter : content)
    {
      auto jter = rhs.content.find(iter.first);
      if (jter == rhs.content.end()) { return false; }
      if ((jter->second) != iter.second) { return false; }
    }
    for (const auto& iter : rhs.content)
    {
      auto jter = content.find(iter.first);
      if (jter == content.end()) { return false; }
      if ((jter->second) != iter.second) { return false; }
    }
    return true;
  }

  public override int GetHashCode()
  {
    return HashCode.Combine(content);
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
  public static bool operator<(const QuadraticNumber& rhs) const;
  public static bool operator>(const QuadraticNumber& rhs) const;

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

