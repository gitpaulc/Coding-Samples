/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#ifndef PLATONIC_SOLIDS_H
#define PLATONIC_SOLIDS_H

#include "biquadratic_number.h"
#include "dynamic_matrix.h"

#include <set>

namespace FunctionalCalculator
{
  /** \return Set of vertices (x, y) making up a regular polygon with nn edges.
   *  \remark A regular polygon is a polygon where all the edges have equal length.
   *  \throw  Throws an invalid argument exception if the number of edges is less than or equal to 2.
   *  \throw  Throws an exception if calculating the sine or cosine of (2 * pi / nn) is unsupported.
   *  \param `edgeLength` the length of an edge of the polygon. This is set to 1 by default.
   *  \param `generator` This is an optional output parameter. If it is not the null pointer, outputs a generator for the
   *          group of nn rotations which are symmetries of the polygon. The output is only set if no exception is thrown.
   */
  std::vector<Matrix<BiquadraticNumber> > getRegularPolygon(int nn,
    const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)),
    Matrix<BiquadraticNumber>* generator = nullptr);

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

  /** \return Set of vertices (x, y, z) making up an octahedron.
   *  An octahedron is a shape in three-dimensional space consisting of eight equilateral triangles,
   *  each of whose edges coincides with the edge of another equilateral triangle.
   *  \param `edgeLength` the length of an edge of the octahedron.
   */
  std::set<Matrix<BiquadraticNumber> > getOctahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of rotations which preserve an octahedron. Reflections may also be included.
   *  These transformations map (1, 0, 0) to the six vertices making up an octahedron.
   *  They are the smallest set of rotations forming a group (so that compositions thereof remain in the set).
   *  These symmetries also preserve the cube.
   *  \param `includeReflectons` will also include reflections, doubling the size of the set.
   */
  std::set<Matrix<BiquadraticNumber> > getOctahedralSymmetries(bool includeReflections = false);

  /** \return Set of vertices (x, y, z) making up a regular dodecahedron.
   *  A dodecahedron is a shape in three-dimensional space consisting of twelve pentagons,
   *  each of whose edges coincides with the edge of another pentagon.
   *  It is regular if the edge lengths of the pentagons are all equal.
   *  \param `edgeLength` the length of an edge of the dodecahedron.
   * 
   *  \remark This method grows the dodecahedron organically. Once the result is known, it is faster to
   *  cache the result and return it that way. It starts with a pentagon in the plane { z == 0 } and grows
   *  additional pentagonal faces from there, until any further growing results in no new vertices.
   */
  std::set<Matrix<BiquadraticNumber> > getDodecahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  bool test_dodecahedron();
  bool test_platonic();
}

#endif //def PLATONIC_SOLIDS_H
