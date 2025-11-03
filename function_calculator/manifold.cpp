/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#include "manifold.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  unsigned int Manifold::OpenSet::getDimension() const
  {
    unsigned int dim = 0;
    for (const auto& component : inverse)
    {
      auto componentDim = component.getDimension();
      if (componentDim > dim) { dim = componentDim; }
    }
    return dim;
  }

  Manifold::OpenSet Manifold::OpenSet::InverseStereo(bool northPoleRemoved, unsigned int sphereDimension)
  {
    OpenSet invStereo;
    if (sphereDimension == 0)
    {
      PiRational sgn(PiPolynomial(1));
      if (northPoleRemoved) { sgn = -sgn; }
      invStereo.inverse = { RationalFunction(sgn)}; // Constant function maps x to -1 or +1.
      return invStereo;
    }
    AlgebraicPolynomial one(PiPolynomial(1));
    AlgebraicPolynomial sgn(PiPolynomial(1));
    if (northPoleRemoved) { sgn = -sgn; }
    AlgebraicPolynomial sqNorm;
    for (int ii = 0; ii < (int)sphereDimension; ++ii)
    {
      auto xx = AlgebraicPolynomial::x_iToPower(PiPolynomial(1), ii, 1);
      sqNorm = sqNorm + xx * xx;
    }
    AlgebraicPolynomial denom = one - sgn * sqNorm;
    {
      RationalFunction f0(one + sgn * sqNorm, denom);
      invStereo.inverse.push_back(f0);
    }
    for (int ii = 0; ii < (int)sphereDimension; ++ii)
    {
      RationalFunction f0(AlgebraicPolynomial::x_iToPower(PiPolynomial(2), ii, 1), denom);
      invStereo.inverse.push_back(f0);
    }
    return invStereo;
  }

  Manifold Manifold::sphere(unsigned int dimension)
  {
    Manifold sN;
    CoordinateChart north, south;
    north.patch = OpenSet::InverseStereo(false, dimension);
    south.patch = OpenSet::InverseStereo(true, dimension);
    if (dimension == 0)
    {
      sN.atlas.push_back(north);
      sN.atlas.push_back(south);
      return sN;
    }
    AlgebraicPolynomial one(PiPolynomial(1));
    auto x0 = AlgebraicPolynomial::x_iToPower(PiPolynomial(1), 0, 1);
    AlgebraicPolynomial denomN = one + x0;
    AlgebraicPolynomial denomS = one - x0;
    for (int ii = 0; ii < (int)dimension; ++ii)
    {
      auto x_iPlus1 = AlgebraicPolynomial::x_iToPower(PiPolynomial(1), ii + 1, 1);
      north.mapping.push_back(RationalFunction(x_iPlus1, denomN));
      south.mapping.push_back(RationalFunction(x_iPlus1, denomS));
    }
    sN.atlas.push_back(north);
    sN.atlas.push_back(south);
    return sN;
  }
}
