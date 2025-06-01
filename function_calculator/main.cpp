
#include <iostream>

#include "function.h"
#include "mp_integer.h"
#include "pi_polynomial.h"

using namespace FunctionalCalculator;

bool test_mp()
{
  mp zero = 0;
  std::cout << "\n0 = " << zero;
  mp one = 1;
  mp two = one + one;
  std::cout << "\n1 = " << one << "\n2 = " << two;
  mp twoToThe16 = two.pow(16);
  std::cout << "\n65,536 = " << twoToThe16;
  mp thousand = 1000;
  std::cout << "\n1000 = " << thousand;
  auto million = mp(500000) + mp(500000);
  std::cout << "\n1 million = " << million;
  million = thousand * thousand;
  std::cout << "\n1 million = " << million;
  std::cout << "\n1,000,001 = " << (million + one);
  std::cout << "\n1 trillion = " << million * million;
  auto squareOf_65536 = (twoToThe16 * twoToThe16);
  std::cout << "\n65,536^2 = " << squareOf_65536;
  auto squareOf_8192 = mp(8192) * mp(8192);
  std::cout << "\n8192^2 = " << (squareOf_8192);
  std::cout << "\n6th digit of 8192^2 = " << squareOf_8192.getDigit(6);
  auto replaced = squareOf_8192;
  replaced.setDigit(6, 5);
  std::cout << "\nReplace 6th digit to 5 in 8192^2 = " << replaced;
  replaced = squareOf_8192;
  replaced.setDigit(7, 5);
  std::cout << "\nReplace 7th digit to 5 in 8192^2 = " << replaced;
  std::cout << "\nNumber of digits in " << squareOf_65536 << " = " << squareOf_65536.numDigits();
  std::cout << "\n10^6 - 500,000 = " << million - mp(500000);
  std::cout << "\n500,000 - 10^6 = " << mp(500000) - million;
  {
    mp divisor(2);
    for (int ii = 31; ii >= 0; --ii)
    {
      std::cout << "\n2^" << ii << " = " << squareOf_65536 / divisor;
      std::cout << " with a remainder of " << squareOf_65536 % divisor;
      divisor = divisor * mp(2);
    }
  }
  std::cout << "\n" << squareOf_8192 << " / " << mp(11) << " = " << (squareOf_8192 / mp(11)) << " with a remainder of " << (squareOf_8192 % mp(11));
  return true;
}

bool test_rational()
{
  Rational zero;
  std::cout << "\nZero = " << zero.print();
  Rational one;
  one = one + Rational(1, 2);
  one = one + Rational(1, 3);
  one = one + Rational(1, 6);
  std::cout << "\nOne = " << one.print();
  std::cout << "\nPrime factorization of one = " << one.printFactors();
  std::cout << "\nPrime factorization of -1 = " << (-one).printFactors();
  Rational half = Rational(-1, 4) * Rational(4, -2);
  std::cout << "\nOne half = " << half.print();
  Rational thePower = half;
  for (int i = 0; i < 6; ++i)
  {
    auto newPower = thePower * thePower;
    std::cout << "\n" << thePower.print() << "^2 = " << newPower.print();
    thePower = newPower;
  }
  Rational twelve = Rational(36, 3);
  std::cout << "\nPrime factorization of twelve = " << twelve.printFactors();
  Rational minusTwelve = Rational(24, -2);
  std::cout << "\nPrime factorization of negative twelve = " << minusTwelve.printFactors();
  Rational oneOver2048 = Rational(2, 4096);
  std::cout << "\nPrime factorization of 1 / 2048 = " << oneOver2048.printFactors();
  Rational hundred = Rational(1000, 10);
  std::cout << "\nPrime factorization of 100 = " << hundred.printFactors();
  Rational myNum = Rational(-24, 138);
  std::cout << "\nPrime factorization of -24 / 138 = " << myNum.printFactors();
  return true;
}

bool test_quadratic()
{
  auto zero = QuadraticNumber();
  std::cout << "\nZero = " << zero.print();
  zero = QuadraticNumber::sqrt(9) - Rational(3);
  std::cout << "\nZero = " << zero.print();
  auto twoThirds = QuadraticNumber(Rational(2, 3));
  std::cout << "\nTwo-thirds = " << twoThirds.print();
  auto one = QuadraticNumber::sqrt(1);
  std::cout << "\nSquare root of 1 = " << one.print();
  auto sqrt2 = QuadraticNumber::sqrt(2);
  std::cout << "\nSquare root of 2 = " << sqrt2.print();
  auto sqrt36 = QuadraticNumber::sqrt(36);
  std::cout << "\nSquare root of 36 = " << sqrt36.print();
  auto sqrt12 = QuadraticNumber::sqrt(12);
  std::cout << "\nSquare root of 12 = " << sqrt12.print();
  std::cout << "\nTwelve is " << (sqrt12 * sqrt12).print();
  Rational rationalOut;
  bool twoThirdsIsRational = twoThirds.getRational(rationalOut);
  if (!twoThirdsIsRational) { return false; }
  std::cout << "\nSquare root of 2/3 = " << QuadraticNumber::sqrt(rationalOut).print();
  auto goldenRatio = QuadraticNumber::sqrt(Rational(5, 4)) + Rational(1, 2);
  std::cout << "\nThe golden ratio is " << goldenRatio.print();
  auto oneOverGolden = QuadraticNumber::sqrt(Rational(5, 4)) - Rational(1, 2);
  std::cout << "\nOne = " << (goldenRatio * oneOverGolden).print();
  oneOverGolden = QuadraticNumber(1) / goldenRatio;
  std::cout << "\nThe reciprocal golden ratio is " << oneOverGolden.print();
  auto sumOfSquareRoots = QuadraticNumber::sqrt(2) + QuadraticNumber::sqrt(3) + Rational(1);
  auto reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\nThe reciprocal of " << sumOfSquareRoots.print() << " is " << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();
  sumOfSquareRoots = QuadraticNumber::sqrt(5) - QuadraticNumber::sqrt(3) + Rational(1);
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();
  return true;
}

bool test_complex()
{
  auto ii = ComplexQuadratic::sqrt(-1);
  std::cout << "\nSquare root of -1 = " << ii.print();
  auto sqrtI = ComplexQuadratic::sqrtOfITimes(1);
  std::cout << "\nSquare root of i = " << sqrtI.print();
  std::cout << "\ni = " << (sqrtI * sqrtI).print();
  auto sqrtMinus12 = ComplexQuadratic::sqrt(-12);
  std::cout << "\nSquare root of -12 = " << sqrtMinus12.print();
  auto sqrtMinus36 = ComplexQuadratic::sqrt(-36);
  std::cout << "\nSquare root of -36 = " << sqrtMinus36.print();
  std::cout << "\nThe reciprocal of i is " << (ComplexQuadratic::sqrt(-1).pow(-1)).print();
  auto rootThreeNum = (ComplexQuadratic::sqrt(-3) + QuadraticNumber(1)) / QuadraticNumber(2);
  std::cout << "\nThe following equation holds:\n" << rootThreeNum.print(true) << " * " << rootThreeNum.conjugate().print(true);
  std::cout << " = " << (rootThreeNum * rootThreeNum.conjugate()).print();
  return true;
}

bool test_pi()
{
  auto piPoly = PiPolynomial();
  std::cout << "\n0 * pi^0 = " << piPoly.print();
  piPoly = PiPolynomial(ComplexQuadratic::sqrt(-1), 2);
  std::cout << "\ni * pi^2 = " << piPoly.print();
  piPoly = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(1))});
  auto piPoly1 = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(-1))});
  auto piPoly2 = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(0)), ComplexQuadratic(Rational(1))});
  std::cout << "\n1 + pi = " << piPoly.print();
  std::cout << "\n1 - pi = " << piPoly1.print();
  std::cout << "\n1 + pi^2 = " << piPoly2.print();
  auto oneMinusPiSq = piPoly * piPoly1;
  std::cout << "\n1 - pi^2 = " << oneMinusPiSq.print();
  auto product = piPoly * piPoly1 * piPoly2;
  std::cout << "\n1 - pi^4 = " << product.print();
  auto one = product + PiPolynomial(ComplexQuadratic(1), 4);
  std::cout << "\n1 = " << one.print();
  PiPolynomial remainder;
  PiPolynomial divisor = one + PiPolynomial(2, 3);
  auto quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  divisor = piPoly1;
  quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  product = PiPolynomial(4) - PiPolynomial(9, 4);
  divisor = PiPolynomial(ComplexQuadratic::sqrt(2)) - PiPolynomial(ComplexQuadratic::sqrt(3), 1);
  quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  product = piPoly * piPoly2;
  std::cout << "\n\nThe gcd of " << product.print() << " and " << oneMinusPiSq.print();
  std::cout << " is " << PiPolynomial::gcd(product, oneMinusPiSq).print();
  std::cout << "\nThe gcd of " << oneMinusPiSq.print() << " and " << product.print();
  std::cout << " is " << PiPolynomial::gcd(oneMinusPiSq, product).print();
  return true;
}

bool test_fn_poly()
{
  {
    FnPolynomial sineOfPiX = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    std::cout << "\n\nsin(pi * x) = " << sineOfPiX.print();
    FnPolynomial piCosPiX = sineOfPiX.partial_x();
    std::cout << "\npi * cos(pi * x) = " << piCosPiX.print();

    // The calculator deduces sin^2 + cos^2 = 1:
    auto one = sineOfPiX * sineOfPiX + (piCosPiX * piCosPiX) * (PiRational(ComplexQuadratic(1), PiPolynomial(1, 2)));
    std::cout << "\n\nsin^2(pi * x) + cos^2(pi * x) = " << one.print();
  }
  {
    std::cout << "\n\nTrig identities:";
    FnPolynomial sinPi2X = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 2);
    FnPolynomial cosPi2X = FnPolynomial::cosATimesPiX(PiPolynomial(ComplexQuadratic(1)), 2);
    FnPolynomial sinPiX = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    FnPolynomial cosPiX = FnPolynomial::cosATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    auto identity1 = sinPi2X * (PiPolynomial(1) * Rational(1, 2)) - sinPiX * cosPiX;
    auto identity2 = cosPi2X - cosPiX * cosPiX + sinPiX * sinPiX;
    std::cout << "\nsin(2 * pi * x) / 2 - sin(pi * x) * cos(pi * x) = " << identity1.print();
    std::cout << "\ncos(2 * pi * x) - cos^2(pi * x) + sin^2(pi * x) = " << identity2.print();
  }
  {
    auto harmonic = FnPolynomial::eToTheATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiY(PiPolynomial(2), 3);
    std::cout << "\n\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    auto notHarmonic = FnPolynomial::eToTheATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiX(PiPolynomial(2), 3);
    std::cout << "\nThe function " << notHarmonic.print() << " is " << (notHarmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  {
    auto efunc = FnPolynomial::sinATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiY(PiPolynomial(2), 3);
    PiRational lambda;
    bool isEigen = efunc.isLaplaceEigenfunction(lambda);
    if (isEigen)
    {
      std::cout << "\n\nThe function " << efunc.print() << " is a Laplace eigenfunction with eigenvalue " << lambda.print() << ".";
    }
    auto notEfunc = efunc + FnPolynomial::sinATimesPiX(PiPolynomial(2), 3);
    PiRational shouldNotChange;
    isEigen = notEfunc.isLaplaceEigenfunction(shouldNotChange);
    if (!isEigen) { std::cout << "\n\nThe function " << notEfunc.print() << " is not a Laplace eigenfunction."; }
  }
  std::cout << "\n";
  return true;
}

bool test_function()
{
  auto xx = FnPolynomial::xToPower(PiPolynomial(1), 1);
  auto yy = FnPolynomial::yToPower(PiPolynomial(1), 1);
  std::cout << "\n";
  {
    auto tan = Function::tanATimesPiX(PiPolynomial(1), 1);
    auto sec = Function(FnPolynomial(PiPolynomial(1)), FnPolynomial::cosATimesPiX(PiPolynomial(1), 1));
    auto sec2_times_pi = sec * sec * Function::constant(PiPolynomial(1, 1));
    std::cout << "\npi * sec^2(pi * x) - (d/dx)tan(pi * x) = " << (sec2_times_pi - tan.partial_x()).print();
  }
  {
    auto harmonic = Function(xx * xx - yy * yy);
    std::cout << "\n\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    harmonic = Function(xx, xx * xx + yy * yy);
    std::cout << "\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    auto notHarmonic = Function(xx, xx * xx - yy * yy);
    std::cout << "\nThe function " << notHarmonic.print() << " is " << (notHarmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  {
    auto numerator = FnPolynomial::sinATimesPiX(PiPolynomial(1), 1) * FnPolynomial::cosATimesPiX(PiPolynomial(1), 1);
    auto coshTerm = FnPolynomial::coshATimesPiY(PiPolynomial(1), 1) * FnPolynomial::coshATimesPiY(PiPolynomial(1), 1);
    auto sinTerm = FnPolynomial::sinATimesPiX(PiPolynomial(1), 1) * FnPolynomial::sinATimesPiX(PiPolynomial(1), 1);
    auto denominator = coshTerm - sinTerm;
    auto harmonic = Function(numerator, denominator);
    std::cout << "\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  return true;
}

int main()
{
  std::string prompt;
  std::cout << "\n\nTesting functions:\n";
  test_function();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest function polynomials:\n";
  test_fn_poly();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\nTest rational:\n";
  test_rational();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest quadratic:\n";
  test_quadratic();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest complex:\n";
  test_complex();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest pi polynomials:\n";
  test_pi();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTesting multiprecision integers:\n";
  test_mp();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
}
