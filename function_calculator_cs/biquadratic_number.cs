/*  Copyright Paul Cernea, 2026.
All Rights Reserved.*/

namespace function_calculator_cs
{

public class BiquadraticNumber
{
  private SortedDictionary<QuadraticNumber, QuadraticNumber> content =
    new SortedDictionary<QuadraticNumber, QuadraticNumber>();

  public BiquadraticNumber() { }

  public BiquadraticNumber(Rational number) : this(new QuadraticNumber(number)) { }

  public BiquadraticNumber(QuadraticNumber number)
  {
    if (number != QuadraticNumber.zero())
    {
      var one_ = new QuadraticNumber(new Rational(1));
      content[one_] = number;
    }
  }

  public BiquadraticNumber(BiquadraticNumber other)
  {
    foreach (var kv in other.content) { content[kv.Key] = kv.Value; }
  }
}
}
