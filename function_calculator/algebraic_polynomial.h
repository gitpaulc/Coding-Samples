/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#ifndef ALGEBRAIC_POLYNOMIAL_H
#define ALGEBRAIC_POLYNOMIAL_H

#include "dynamic_matrix.h"
#include "pi_rational.h"

#include <map>
#include <vector>

namespace FunctionalCalculator
{

/** \class Represents a polynomial in x_0, x_1, ..., x_{n - 1}.
 */
class AlgebraicPolynomial
{

  struct Monomial
  {
    /** The key is the index of the variable, the value is the power of the variable. */
    std::map<unsigned int, unsigned int> indices;
    bool isConstTerm() const;
    unsigned int getDimension() const;
    Monomial operator*(const Monomial& rhs) const;
    bool operator<(const Monomial& rhs) const;
    void clean();
  };

  std::map<Monomial, PiRational> self;
  void clean();
public:
  AlgebraicPolynomial(const PiRational& coeff = PiPolynomial(0));
  unsigned int getDimension() const;
  virtual std::string print(bool useParentheses = false, bool detectLowDimension = true) const;

  // MATRIX COMPOSITION:

  /** \brief If the poly. is F(u), returns F(M * u) where M is the matrix and u is (x_0, x_1, ..., x_{n-1}). */
  // TODO: AlgebraicPolynomial composeWith(const Matrix<ComplexQuadratic>& transform) const;

  // POLYNOMIALS:

  /**< \return coeff * x_i^p where p is the power. */
  static AlgebraicPolynomial x_iToPower(const PiRational& coeff, unsigned int i, unsigned int p);
  static AlgebraicPolynomial xToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * x^p */
  static AlgebraicPolynomial yToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * y^p */
  static AlgebraicPolynomial zToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * z^p */
  static AlgebraicPolynomial wToPower(const PiRational& coeff, unsigned int p); /**< \return coeff * w^p */

  AlgebraicPolynomial operator+() const;
  AlgebraicPolynomial operator-() const;
  AlgebraicPolynomial operator+(const AlgebraicPolynomial& rhs) const;
  AlgebraicPolynomial operator-(const AlgebraicPolynomial& rhs) const;
  AlgebraicPolynomial operator*(const AlgebraicPolynomial& rhs) const;
  AlgebraicPolynomial operator*(const PiPolynomial& rhs) const;
  AlgebraicPolynomial pow(int p) const; /**< `return` The p'th power of the polynomial. */
  bool operator==(const AlgebraicPolynomial& rhs) const;
  bool operator!=(const AlgebraicPolynomial& rhs) const;

  AlgebraicPolynomial partial_deriv(unsigned int index) const;
  AlgebraicPolynomial partial_x() const;
  AlgebraicPolynomial partial_y() const;
  AlgebraicPolynomial partial_z() const;
  AlgebraicPolynomial partial_w() const;
  AlgebraicPolynomial laplacian() const;

  bool isLaplaceEigenfunction(PiRational& eigenvalue) const;
  bool isHarmonic() const;

  // EXACT EVALUATION:

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   */
  bool tryEvaluate(const std::vector<PiRational>& input, AlgebraicPolynomial& output) const;
};
}

#endif //def ALGEBRAIC_POLYNOMIAL_H
