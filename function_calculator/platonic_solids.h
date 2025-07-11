/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#ifndef PLATONIC_SOLIDS_H
#define PLATONIC_SOLIDS_H

#include "biquadratic_number.h"
#include "dynamic_matrix.h"

#include <set>

namespace FunctionalCalculator
{
  /** \return Set of vertices (x, y, z) making up a tetrahedron.
   *  A (regular) tetrahedron is a shape in three-dimensional space consisting of
   *  four points of equal distance from one another.
   *  The faces, therefore, must be equilateral triangles (triangles each of whose edges have equal length).
   *  \param `edgeLength` the length of an edge of the tetrahedron.
   */
  std::set<Matrix<BiquadraticNumber> > getTetrahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of rotations which preserve a (regular) tetrahedron. Reflections may also be included.
   *  These transformations map (1, 0, 0) to the four vertices making up a tetrahedron.
   *  They are the smallest set of rotations forming a group (so that compositions thereof remain in the set).
   *  \param `includeReflectons` will also include reflections, doubling the size of the set.
   */
  std::set<Matrix<BiquadraticNumber> > getTetrahedralSymmetries(bool includeReflections = false);

  /** \return Set of vertices (x, y, z) making up a cube.
   *  A cube is a shape in three-dimensional space consisting of six squares,
   *  each of whose edges coincides with the edge of another square.
   *  \param `edgeLength` the length of an edge of the cube.
   */
  std::set<Matrix<BiquadraticNumber> > getCube(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of rotations which preserve a cube. Reflections may also be included.
   *  These transformations map (1, 1, 1) to the eight vertices making up a cube.
   *  They are the smallest set of rotations forming a group (so that compositions thereof remain in the set).
   *  These symmetries are called octahedral symmetries because they also preserve the octahedron.
   *  \param `includeReflectons` will also include reflections, doubling the size of the set.
   */
  std::set<Matrix<BiquadraticNumber> > getSymmetriesOfACube(bool includeReflections = false);

  bool test_platonic();
}

#endif //def PLATONIC_SOLIDS_H
