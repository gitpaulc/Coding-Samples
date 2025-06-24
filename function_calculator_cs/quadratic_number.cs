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
  std::pair<QuadraticNumber, mp> factorAsIntegral() const;
  bool getRational(Rational& self) const; /**< \return `true` iff the number is actually rational. Only then is self redefined. */
  virtual std::string print(bool useParentheses = false) const override;
  static QuadraticNumber sqrt(const Rational& radicand);
  QuadraticNumber abs() const;

  QuadraticNumber operator+() const;
  QuadraticNumber operator-() const;
  QuadraticNumber operator+(const QuadraticNumber& rhs) const;
  QuadraticNumber operator-(const QuadraticNumber& rhs) const;
  QuadraticNumber operator*(const QuadraticNumber& rhs) const;
  /** \brief Uses inversion of the multiplication operator. */
  QuadraticNumber operator/(const QuadraticNumber& rhs) const;
  QuadraticNumber pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const QuadraticNumber& rhs) const;
  bool operator!=(const QuadraticNumber& rhs) const;
  bool operator!=(int rhs) const;
  /** \remark Does not use algebra to determine < since it would be very inefficient. */
  bool operator<(const QuadraticNumber& rhs) const;
  bool operator>(const QuadraticNumber& rhs) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  //static bool tryGetCosine(const Rational& input, QuadraticNumber& output);

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  //static bool tryGetSine(const Rational& input, QuadraticNumber& output);
}
}

