/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#ifndef MANIFOLD_H
#define MANIFOLD_H

#include <string>

#include "rational_function.h"

namespace FunctionalCalculator
{

/** \class Represents a manifold such as the n-dimensional sphere, torus, or projective space.
 *  The Riemannian geometry does not necessarily coincide with that of the ambient space in which the manifold is
 *  embedded.
 *  According to the Nash Embedding Theorem the Riemannian geometry can always be isometrically embedded in some
 *  high-enough dimensional space, though this cannot always be computed directly.
 */
class Manifold
{
public:
  /** \class Represents an open set in the manifold's topology. Such open sets should cover the manifold.
   *  For example, the 2-sphere minus the north pole and the 2-sphere minus the south pole within 3d Euclidean space.
   *  Notice that those examples are not open sets in 3d space, but they are open sets in the manifold topology.
   */
  class OpenSet
  {
  public:
    /** \brief This function F = (f_0, f_1, ... f_K) inversely maps its domain in R^n to the open set in the n-dimensional manifold. */
    std::vector<RationalFunction> inverse;
    /** \brief This returns 3 if we have a function of (y, z) without x so care must be taken for Cartesian products, etc. */
    unsigned int getDimension() const;
    /** \return The inverse stereographic projection. If `northPoleRemoved` is false then the south pole is removed. */
    static OpenSet InverseStereo(bool northPoleRemoved, unsigned int sphereDimension);
  };
  class CoordinateChart
  {
  public:
    OpenSet patch;
    /** \brief This mapping restricts to an invertible smooth mapping on the coordinate patch. */
    std::vector<RationalFunction> mapping;
  };
private:
  std::vector<CoordinateChart> atlas; /**< This is not the (infinite) maximal atlas. Rather it is a finite cover. */
public:
  static Manifold sphere(unsigned int dimension); /**< \return S^n where the dimension == n. */
};
}

#endif //def MANIFOLD_H
