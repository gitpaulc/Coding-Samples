
#include <iostream>

#include "complex_quadratic.h"

using namespace FunctionalCalculator;

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
}

int main()
{
  std::string prompt;
  std::cout << "\nTest rational:\n";
  test_rational();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  std::cout << "\n\nTest quadratic:\n";
  test_quadratic();
  std::cout << "\nContinue... ";
  std::cin >> prompt;
  std::cout << "\n\nTest complex:\n";
  test_complex();
  std::cout << "\nContinue... ";
}
