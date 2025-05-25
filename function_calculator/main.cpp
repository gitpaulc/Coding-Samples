
#include <iostream>

#include "rational.h"

using namespace FunctionalCalculator;

bool test_1()
{
  Rational zero;
  std::cout << "\nZero = " << zero.print();
  Rational one;
  one = one + Rational(1, 2);
  one = one + Rational(1, 3);
  one = one + Rational(1, 6);
  std::cout << "\nOne = " << one.print();
  Rational half = Rational(-1, 4) * Rational(4, -2);
  std::cout << "\nOne half = " << half.print();
  Rational twelve = Rational(36, 3);
  std::cout << "\nPrime factorization of twelve = " << twelve.printFactors();
  return true;
}

int main()
{
  test_1();
}
