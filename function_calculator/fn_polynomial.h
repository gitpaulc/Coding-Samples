/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FN_POLYNOMIAL_H
#define FN_POLYNOMIAL_H

#include "dynamic_matrix.h"
#include "pi_rational.h"

#include <map>
#include <vector>

namespace FunctionalCalculator
{

/** \class Represents a polynomial in x, y, z, and e^{pi * (a * x + b * y + c * z)}.
 *
 *  \remark In fact, the powers of x, y, and z are allowed to be negative.
 *
 *  Here a, b, and c are of the form A + B * sqrt(d) where d is an integer. The coefficients of
 *  the polynomial are rational functions of pi and A + B * sqrt(d) where d is an integer. Here d can be -1.
 * 
 *  \remark This illustrates a practical application of the fact that sqrt(d) is nonrational if d is an
 *  integer that is not a perfect square, and that pi is transcendental: It means we can check for exact equality
 *  by checking that a polynomial in one of these variables is equal to zero.
 */
class FnPolynomial
{
  /** \param `self` represents A in sin(pi * A * x) where A != 0 OR A in cos(pi * A * x) where A might be 0. */
  struct TrigIndex
  {
    BiquadraticNumber self;
    bool isCosine = true; /**< Is cosine if and only if: isCosine == true OR self == 0. Otherwise is sine. */
    bool isCos() const;
    bool operator==(const TrigIndex& rhs) const;
    bool operator!=(const TrigIndex& rhs) const;
    bool operator<(const TrigIndex& rhs) const;
    bool operator>(const TrigIndex& rhs) const;
  };

  struct Monomial
  {
    unsigned int xInd = 0;
    unsigned int yInd = 0;
    unsigned int zInd = 0;
    BiquadraticNumber ePiXInd;
    BiquadraticNumber ePiYInd;
    BiquadraticNumber ePiZInd;
    TrigIndex trigPiXInd;
    TrigIndex trigPiYInd;
    TrigIndex trigPiZInd;
    bool isConstTerm() const;
    std::map<Monomial, BiquadraticNumber> trigSum(const Monomial& rhs) const;
    bool operator<(const Monomial& rhs) const;
  };

  /** \brief Seeks duplicated Monomial index since sin(-ax) == -sin(ax) and cos(-ax) == cos(ax).
   *  \remark Note that sin(ax) == sin(Ax) implies a == A, and cos(ax) == cos(Ax) implies |a| = |A|.
   *  \return self.end() if not found.
   */
  std::map<Monomial, PiRational>::iterator trigFind(const Monomial& ind, bool& xNegative, bool& yNegative, bool& zNegative);

  std::map<Monomial, PiRational> self;
  void clean();
public:
  FnPolynomial(const PiRational& coeff = PiPolynomial(0));
  virtual std::string print(bool useParentheses = false) const;

  // MATRIX COMPOSITION:

  /** \brief If the function is F(u), returns F(M * u) where M is the matrix and u is a 3d vector (x, y, z). */
  FnPolynomial composeWith(const Matrix<ComplexQuadratic>& transform) const;

  // POLYNOMIALS:

  static FnPolynomial xToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * x^p */
  static FnPolynomial yToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * y^p */
  static FnPolynomial zToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * z^p */
  /** \brief \return (A * x + B * y + C * z + D)^p */
  static FnPolynomial multinomial(const PiRational& coeff,
      const PiRational& A, const PiRational& B, const PiRational& C, const PiRational& D, unsigned int p);

  // EXPONENTIALS:

  static FnPolynomial eToTheATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * x} */
  static FnPolynomial eToTheATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * y} */
  static FnPolynomial eToTheATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * z} */
  /** \brief \return e^{Pi * (A * x + B * y + C * z)} */
  static FnPolynomial eToThePi_AX_plus_BY_plus_CZ(const PiRational& coeff,
    const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);

  // TRIG FUNCTIONS:

  static FnPolynomial sinATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * x) */
  static FnPolynomial sinATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * y) */
  static FnPolynomial sinATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * z) */
  /** \brief \return sin(Pi * (A * x + B * y + C * z)) */
  static FnPolynomial sinPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);
  static FnPolynomial cosATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * x) */
  static FnPolynomial cosATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * y) */
  static FnPolynomial cosATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * z) */
  /** \brief \return cos(Pi * (A * x + B * y + C * z)) */
  static FnPolynomial cosPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);

  // HYPERBOLIC FUNCTIONS:

  static FnPolynomial sinhATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sinh(A * Pi * x) */
  static FnPolynomial sinhATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sinh(A * Pi * y) */
  static FnPolynomial sinhATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * sinh(A * Pi * z) */
  /** \brief \return sinh(Pi * (A * x + B * y + C * z)) */
  static FnPolynomial sinhPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);
  static FnPolynomial coshATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cosh(A * Pi * x) */
  static FnPolynomial coshATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cosh(A * Pi * y) */
  static FnPolynomial coshATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * cosh(A * Pi * z) */
  /** \brief \return cosh(Pi * (A * x + B * y + C * z)) */
  static FnPolynomial coshPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);

  FnPolynomial operator+() const;
  FnPolynomial operator-() const;
  FnPolynomial operator+(const FnPolynomial& rhs) const;
  FnPolynomial operator-(const FnPolynomial& rhs) const;
  FnPolynomial operator*(const FnPolynomial& rhs) const;
  FnPolynomial operator*(const PiPolynomial& rhs) const;
  FnPolynomial pow(int p) const; /**< `return` The p'th power of the polynomial. */
  bool operator==(const FnPolynomial& rhs) const;
  bool operator!=(const FnPolynomial& rhs) const;

  FnPolynomial partial_x() const;
  FnPolynomial partial_y() const;
  FnPolynomial partial_z() const;
  FnPolynomial laplacian() const;

  bool isLaplaceEigenfunction(PiRational& eigenvalue) const;
  bool isHarmonic() const;

  // EXACT EVALUATION:

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real BiquadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtX(const ComplexQuadratic& xVal, FnPolynomial& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real BiquadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtY(const ComplexQuadratic& yVal, FnPolynomial& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real BiquadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtZ(const ComplexQuadratic& zVal, FnPolynomial& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real BiquadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtXYZ(const ComplexQuadratic& xVal, const ComplexQuadratic& yVal, const ComplexQuadratic& zVal, PiRational& output) const;

  friend class Function;
};
}

#endif //def FN_POLYNOMIAL_H
