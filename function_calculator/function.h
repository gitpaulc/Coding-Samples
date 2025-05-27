/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FUNCTION_H
#define FUNCTION_H

#include <string>

#include "fn_polynomial.h"

namespace FunctionalCalculator
{

/** \class Represents a rational function of x, y, z, e^{pi * x}, e^{pi * y}, e^{pi * z} where x, y, and z are rational complex numbers.
 * 
 *  The transcendental functions e^{pi * x}, e^{pi * y}, e^{pi * z} can effectively be treated as separate
 *  variables in the rational function, giving us a rational function in 6 variables. The coefficients of
 *  the rational function are polynomials in pi with coefficients that are complex rational functions of
 *  square roots of rational numbers.
 * 
 *  \remark This illustrates a practical application of the fact that sqrt(d) is nonrational if d is an
 *  integer that is not a perfect square, and that pi is transcendental: It means we can check for exact equality
 *  by checking that a polynomial in one of these variables is equal to zero.
 */
class Function
{
  FnPolynomial num, den;

public:
  Function();
};
}

#endif //def FUNCTION_H
