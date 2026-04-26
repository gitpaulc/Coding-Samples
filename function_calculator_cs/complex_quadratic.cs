/*  Copyright Paul Cernea, 2026.
All Rights Reserved.*/

namespace function_calculator_cs
{

public class ComplexQuadratic
{
  private BiquadraticNumber re;
  private BiquadraticNumber im;

  public ComplexQuadratic(int reIn = 0)
  {
    re = new BiquadraticNumber(new Rational(reIn, 1));
    im = new BiquadraticNumber();
  }

  public ComplexQuadratic(BiquadraticNumber reIn, BiquadraticNumber? imIn = null)
  {
    re = reIn;
    im = imIn ?? new BiquadraticNumber();
  }
}
}
