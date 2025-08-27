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

  /** \brief Exports a tetrahedron as an .obj file. It has unit edges and is centered at the origin.
   *  \return `true` if and only if the export succeeds.
   *  \param `filename` is the name of the file the user wishes to export the shape to.
   */
  bool exportTetrahedronObj(const std::string& filename);

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

  /** \brief Exports a cube as an .obj file. It has unit edges and is centered at the origin.
   *  \return `true` if and only if the export succeeds.
   *  \param `filename` is the name of the file the user wishes to export the shape to.
   */
  bool exportCubeObj(const std::string& filename);

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
   *  \remark This method grows the dodecahedron organically. 
   *  It starts with a pentagon in the plane { z == 0 } and grows
   *  additional pentagonal faces from there, until any further growing results in no new vertices.
   *  \remark This method is fairly slow, and if efficiency is important, use getDodecahedron.
   */
  std::set<Matrix<BiquadraticNumber> > growDodecahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of vertices (x, y, z) making up a regular dodecahedron.
   *  A dodecahedron is a shape in three-dimensional space consisting of twelve pentagons,
   *  each of whose edges coincides with the edge of another pentagon.
   *  It is regular if the edge lengths of the pentagons are all equal.
   *  \param `edgeLength` the length of an edge of the dodecahedron.
   *
   *  \remark This method uses values cached from an initial run of growDodecahedron and is fast.
   */
  std::set<Matrix<BiquadraticNumber> > getDodecahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \brief Exports a regular dodecahedron as an .obj file. It has unit edges and is centered at the origin.
   *  \return `true` if and only if the export succeeds.
   *  \param `filename` is the name of the file the user wishes to export the shape to.
   */
  bool exportDodecahedronObj(const std::string& filename);

  /** \return Set of vertices (x, y, z) making up a regular icosahedron.
   *  An icosahedron is a shape in three-dimensional space consisting of twenty triangles,
   *  each of whose edges coincides with the edge of another triangle.
   *  The icosahedron is regular if all the triangles are equilateral, i.e. have the same edge length.
   *  \param `edgeLength` the length of an edge of the icosahedron.
   *  \remark This method uses a dual dodecahedron to obtain the vertices of the icosahedron, and is slow.
   */
  std::set<Matrix<BiquadraticNumber> > getIcosahedronViaDual(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of vertices (x, y, z) making up a regular icosahedron.
   *  An icosahedron is a shape in three-dimensional space consisting of twenty triangles,
   *  each of whose edges coincides with the edge of another triangle.
   *  The icosahedron is regular if all the triangles are equilateral, i.e. have the same edge length.
   *  \param `edgeLength` the length of an edge of the icosahedron.
   *  \remark This method uses cached vertex values and is fast.
   */
  std::set<Matrix<BiquadraticNumber> > getIcosahedron(const BiquadraticNumber& edgeLength = BiquadraticNumber(Rational(1)));

  /** \return Set of rotations which preserve an icosahedron. Reflections may also be included.
   *  These transformations map (0, 0, 1) to the twelve vertices making up an icosahedron.
   *  They are the smallest set of such rotations forming a group (so that compositions thereof remain in the set).
   *  These symmetries also preserve the dodecahedron.
   *  \param `includeReflectons` will also include reflections, doubling the size of the set.
   */
  std::set<Matrix<BiquadraticNumber> > getIcosahedralSymmetries(bool includeReflections = false);

  /** \brief Exports a regular icosahedron as an .obj file. It has unit edges and is centered at the origin.
   *  \return `true` if and only if the export succeeds.
   *  \param `filename` is the name of the file the user wishes to export the shape to.
   */
  bool exportIcosahedronObj(const std::string& filename);

  bool test_dodecahedron();
  bool test_icosahedron();
  bool test_icosahedral_symmetries();
  bool test_platonic();
}

#endif //def PLATONIC_SOLIDS_H
