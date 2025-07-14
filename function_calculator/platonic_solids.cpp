/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#include "platonic_solids.h"

#include <iostream>

namespace FunctionalCalculator
{
  std::set<Matrix<BiquadraticNumber> > getRegularPolygon(int nn,
    const BiquadraticNumber& edgeLength, Matrix<BiquadraticNumber>* generator)
  {
    std::set<Matrix<BiquadraticNumber> > polygon;
    if (nn <= 2) { throw std::invalid_argument("The number of edges in the polygon must be greater than 2."); return polygon; }
    auto radius = edgeLength;
    Matrix<BiquadraticNumber> vec0;
    {
      Rational angle(1, nn);
      BiquadraticNumber half(Rational(1, 2));
      BiquadraticNumber sinAngle;
      bool success = BiquadraticNumber::tryGetSine(angle, sinAngle);
      if (!success) { throw std::exception("Unsupported angle."); return polygon; }
      radius = half * radius / sinAngle;
    }
    vec0.addRow({ radius, BiquadraticNumber() });
    vec0 = vec0.transpose();
    for (int ii = 0; ii < nn; ++ii)
    {
      if (ii == 0)
      {
        polygon.insert(vec0);
        continue;
      }
      Rational angle(2, nn);
      BiquadraticNumber cosAngle, sinAngle;
      bool success = BiquadraticNumber::tryGetCosine(angle, cosAngle);
      success = success && BiquadraticNumber::tryGetSine(angle, sinAngle);
      if (!success) { throw std::exception("Unsupported angle."); break; }
      Matrix<BiquadraticNumber> R;
      R.addRow({ cosAngle, sinAngle });
      R.addRow({ -sinAngle, cosAngle });
      polygon.insert(R * vec0);
      if ((generator != nullptr) && (ii == 1))
      {
        *generator = R;
      }
    }
    return polygon;
  }

  std::set<Matrix<BiquadraticNumber> > getTetrahedron(const BiquadraticNumber& edgeLength)
  {
    Matrix<BiquadraticNumber> vec0;
    vec0.addRow({ edgeLength * BiquadraticNumber::sqrt(6) * Rational(1, 4),
      Rational(0), Rational(0) });
    vec0 = vec0.transpose();
    std::set<Matrix<BiquadraticNumber> > tetrahedron;
    auto tetrahedralSymmetries = getTetrahedralSymmetries();
    for (const auto& sym : tetrahedralSymmetries)
    {
      tetrahedron.insert((sym * vec0).transpose());
      if (tetrahedron.size() >= 4) { break; }
    }
    return tetrahedron;
  }

  std::set<Matrix<BiquadraticNumber> > getTetrahedralSymmetries(bool includeReflections)
  {
    std::set<Matrix<BiquadraticNumber> > tetrahedralSymmetries;
    Matrix<BiquadraticNumber> P;
    P.addRow({ Rational(-1, 3), BiquadraticNumber::sqrt(Rational(2, 3)) * Rational(-1), BiquadraticNumber::sqrt(Rational(2)) * Rational(-1, 3) });
    P.addRow({ BiquadraticNumber::sqrt(Rational(2, 3)), Rational(-1, 2) , BiquadraticNumber::sqrt(Rational(1, 3)) * Rational(1, 2) });
    P.addRow({ BiquadraticNumber::sqrt(Rational(2)) * Rational(-1, 3), BiquadraticNumber::sqrt(Rational(1, 3)) * Rational(-1, 2), Rational(5, 6) });
    Matrix<BiquadraticNumber> R;
    R.addRow({ Rational(1), Rational(0), Rational(0) });
    R.addRow({ Rational(0), Rational(-1, 2), BiquadraticNumber::sqrt(3) * Rational(-1, 2) });
    R.addRow({ Rational(0), BiquadraticNumber::sqrt(3) * Rational(1, 2) , Rational(-1, 2) });
    tetrahedralSymmetries.insert(P);
    tetrahedralSymmetries.insert(R);
    Matrix<BiquadraticNumber> A;
    if (includeReflections)
    {
      A.addRow({ Rational(1), Rational(0), Rational(0) });
      A.addRow({ Rational(0), Rational(-1), Rational(0) });
      A.addRow({ Rational(0), Rational(0) , Rational(1) });
      tetrahedralSymmetries.insert(A);
    }
    int limit = includeReflections ? 24 : 12;
    while (tetrahedralSymmetries.size() < limit)
    {
      auto others = tetrahedralSymmetries;
      for (const auto& rot : tetrahedralSymmetries)
      {
        auto rot_ = P * rot;
        others.insert(rot_);
        rot_ = R * rot;
        others.insert(rot_);
        if (includeReflections)
        {
          rot_ = A * rot;
          others.insert(rot_);
        }
      }
      tetrahedralSymmetries = others;
    }
    return tetrahedralSymmetries;
  }

  std::set<Matrix<BiquadraticNumber> > getCube(const BiquadraticNumber& edgeLength)
  {
    Matrix<BiquadraticNumber> vec0;
    auto halfLength = edgeLength * BiquadraticNumber(Rational(1, 2));
    vec0.addRow({ halfLength, halfLength, halfLength });
    vec0 = vec0.transpose();
    std::set<Matrix<BiquadraticNumber> > cube;
    auto octahedralSymmetries = getSymmetriesOfACube();
    for (const auto& sym : octahedralSymmetries)
    {
      cube.insert((sym * vec0).transpose());
      if (cube.size() >= 8) { break; }
    }
    return cube;
  }

  std::set<Matrix<BiquadraticNumber> > getSymmetriesOfACube(bool includeReflections)
  {
    std::set<Matrix<BiquadraticNumber> > octahedralSymmetries;
    Matrix<BiquadraticNumber> P;
    P.addRow({ Rational(0), Rational(0), Rational(1) });
    P.addRow({ Rational(0), Rational(1), Rational(0) });
    P.addRow({ Rational(-1), Rational(0) , Rational(0) });
    Matrix<BiquadraticNumber> R;
    R.addRow({ Rational(1), Rational(0), Rational(0) });
    R.addRow({ Rational(0), Rational(0), Rational(1) });
    R.addRow({ Rational(0), Rational(-1) , Rational(0) });
    Matrix<BiquadraticNumber> Q;
    Q.addRow({ Rational(0), Rational(1), Rational(0) });
    Q.addRow({ Rational(-1), Rational(0), Rational(0) });
    Q.addRow({ Rational(0), Rational(0) , Rational(1) });
    octahedralSymmetries.insert(P);
    octahedralSymmetries.insert(R);
    octahedralSymmetries.insert(Q);
    Matrix<BiquadraticNumber> A;
    if (includeReflections)
    {
      A.addRow({ Rational(-1), Rational(0), Rational(0) });
      A.addRow({ Rational(0), Rational(-1), Rational(0) });
      A.addRow({ Rational(0), Rational(0) , Rational(-1) });
      octahedralSymmetries.insert(A);
    }
    int limit = includeReflections ? 48 : 24;
    while (octahedralSymmetries.size() < limit)
    {
      auto others = octahedralSymmetries;
      for (const auto& rot : octahedralSymmetries)
      {
        auto rot_ = P * rot;
        others.insert(rot_);
        rot_ = R * rot;
        others.insert(rot_);
        rot_ = Q * rot;
        others.insert(rot_);
        if (includeReflections)
        {
          rot_ = A * rot;
          others.insert(rot_);
        }
      }
      octahedralSymmetries = others;
    }
    return octahedralSymmetries;
  }

  std::set<Matrix<BiquadraticNumber> > getOctahedron(const BiquadraticNumber& edgeLength)
  {
    Matrix<BiquadraticNumber> vec0;
    vec0.addRow({ edgeLength * BiquadraticNumber::sqrt(Rational(1, 2)), Rational(0), Rational(0)});
    vec0 = vec0.transpose();
    std::set<Matrix<BiquadraticNumber> > octahedron;
    auto octahedralSymmetries = getOctahedralSymmetries();
    for (const auto& sym : octahedralSymmetries)
    {
      octahedron.insert((sym * vec0).transpose());
      if (octahedron.size() >= 6) { break; }
    }
    return octahedron;
  }

  std::set<Matrix<BiquadraticNumber> > getOctahedralSymmetries(bool includeReflections)
  {
    return getSymmetriesOfACube(includeReflections);
  }

  bool test_platonic()
  {
    std::string prompt;
    auto tetrahedralSymmetries = getTetrahedralSymmetries();
    std::cout << "\nTetrahedral (orientation-preserving) symmetries are:";
    int ind = -1;
    for (const auto& sym : tetrahedralSymmetries)
    {
      ++ind;
      if ((ind % 5 == 0) && (ind > 0))
      {
        std::cout << "\n\nContinue... or 'E' to end printout?  ";
        std::cin >> prompt;
        if ((prompt.compare("E") == 0) || (prompt.compare("e") == 0)) { break; }
      }
      std::cout << "\n" << sym.print(true);
    }
    std::cout << "\nNumber of tetrahedral (orientation-preserving) symmetries: " << tetrahedralSymmetries.size();

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    {
      auto tetrahedralSymmetries0 = getTetrahedralSymmetries(true);
      std::cout << "\nFull group of tetrahedral symmetries is:";
      ind = -1;
      for (const auto& sym : tetrahedralSymmetries0)
      {
        ++ind;
        if ((ind % 5 == 0) && (ind > 0))
        {
          std::cout << "\n\nContinue... or 'E' to end printout?  ";
          std::cin >> prompt;
          if ((prompt.compare("E") == 0) || (prompt.compare("e") == 0)) { break; }
        }
        std::cout << "\n" << sym.print(true);
      }
      std::cout << "\nSize of full group is: " << tetrahedralSymmetries0.size();

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

      Matrix<BiquadraticNumber> vec0;
      vec0.addRow({ BiquadraticNumber::sqrt(6) * Rational(1, 4),
        Rational(0), Rational(0) });
      vec0 = vec0.transpose();
      std::set<Matrix<BiquadraticNumber> > tetrahedron;
      for (const auto& sym : tetrahedralSymmetries0)
      {
        tetrahedron.insert((sym * vec0).transpose());
      }
      std::cout << "\nVertex count of tetrahedron generated by these is: " << tetrahedron.size();

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
    }

    {
      auto edgeLength = BiquadraticNumber::sqrt(Rational(5));
      auto tetrahedron = getTetrahedron(edgeLength);
      std::cout << "\nVertices of tetrahedron of edge length " << edgeLength.print() << " are:";
      for (const auto& vec : tetrahedron)
      {
        std::cout << "\n" << vec.print(true);
      }
      std::cout << "\n";
      for (const auto& vertexA : tetrahedron)
      {
        for (const auto& vertexB : tetrahedron)
        {
          if (vertexA == vertexB) { continue; }
          std::cout << "\nSquared distance to" << vertexB.print(true) << " = " << (vertexA.matrixSqNorm() - vertexA.matrixDot(vertexB) - vertexA.matrixDot(vertexB) + vertexB.matrixSqNorm()).print();
        }
      }

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
    }

    {
      auto octahedralSymmetries0 = getSymmetriesOfACube(true);
      std::cout << "\nFull group of octahedral symmetries is:";
      ind = -1;
      for (const auto& sym : octahedralSymmetries0)
      {
        ++ind;
        if ((ind % 5 == 0) && (ind > 0))
        {
          std::cout << "\n\nContinue... or 'E' to end printout?  ";
          std::cin >> prompt;
          if ((prompt.compare("E") == 0) || (prompt.compare("e") == 0)) { break; }
        }
        std::cout << "\n" << sym.print(true);
      }
      std::cout << "\nSize of full group is: " << octahedralSymmetries0.size();

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

      std::set<Matrix<BiquadraticNumber> > cube = getCube();
      for (const auto& vertex : cube)
      {
        std::cout << "\nCube vertex: " << vertex.print(true);
      }
      std::cout << "\nVertex count of cube is: " << cube.size();

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

      std::set<Matrix<BiquadraticNumber> > octahedron = getOctahedron();
      for (const auto& vertex : octahedron)
      {
        std::cout << "\nOctahedron vertex: " << vertex.print(true);
      }
      std::cout << "\nVertex count of octahedron is: " << octahedron.size();

      std::cout << "\n\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
    }

    ind = -1;
    bool passedClosedness = true;
    for (const auto& sym0 : tetrahedralSymmetries)
    {
      std::set<Matrix<BiquadraticNumber> > newSymmetries;
      for (const auto& sym : tetrahedralSymmetries)
      {
        newSymmetries.insert(sym0 * sym);
      }
      if (newSymmetries == tetrahedralSymmetries) { std::cout << "\n\nThe set of tetrahedral symmetries is closed under multiplication by " << sym0.print(true); }
      else { passedClosedness = false; }
    }
    if (passedClosedness) { std::cout << "\nThe set of tetrahedral symmetries truly forms a group."; }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    {
      std::set<Matrix<BiquadraticNumber> > newSymmetries;
      for (const auto& sym : tetrahedralSymmetries)
      {
        bool success = false;
        auto symInv = sym.inverse(success);
        if (!success) { std::cout << "\nInverse failed!"; continue; }
        auto det = symInv.determinant();
        std::cout << "\n" << symInv.print(true) << "\nIts determinant is " << det.print() << " since it's a rotation.";
        newSymmetries.insert(sym);
      }
      std::cout << "\nNumber of tetrahedral (orientation-preserving) symmetries: " << newSymmetries.size();
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    {
      Matrix<BiquadraticNumber> vec0;
      vec0.addRow({ BiquadraticNumber::sqrt(6) * Rational(1, 4), Rational(0), Rational(0) });
      vec0 = vec0.transpose();
      std::set<Matrix<BiquadraticNumber> > tetrahedron;
      for (const auto& sym : tetrahedralSymmetries) { tetrahedron.insert((sym * vec0).transpose()); }
      std::cout << "\nNumber of vertices in a tetrahedron = " << tetrahedron.size() << "\nWe can choose them to be:";
      for (const auto& vertex : tetrahedron)
      {
        std::cout << "\n" << vertex.print(true);
      }
      for (const auto& vertexA : tetrahedron)
      {
        std::cout << "\n\nSquared length of " << vertexA.print(true) << " = " << vertexA.matrixSqNorm().print();
        for (const auto& vertexB : tetrahedron)
        {
          if (vertexA == vertexB) { continue; }
          std::cout << "\n  Distance to" << vertexB.print(true) << " = " << (vertexA.matrixSqNorm() - vertexA.matrixDot(vertexB) - vertexA.matrixDot(vertexB) + vertexB.matrixSqNorm()).print();
        }
      }
    }

    std::cout << "\n";
    return true;
  }
}
