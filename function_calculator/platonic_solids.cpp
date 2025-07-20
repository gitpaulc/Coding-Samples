/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#include "platonic_solids.h"

#include <iostream>

namespace FunctionalCalculator
{
  std::vector<Matrix<BiquadraticNumber> > getRegularPolygon(int nn,
    const BiquadraticNumber& edgeLength, Matrix<BiquadraticNumber>* generator)
  {
    std::vector<Matrix<BiquadraticNumber> > polygon;
    if (nn <= 2) { throw std::invalid_argument("The number of edges in the polygon must be greater than 2."); return polygon; }
    auto radius = edgeLength;
    polygon.reserve(nn);
    {
      Rational angle(1, nn);
      BiquadraticNumber half(Rational(1, 2));
      BiquadraticNumber sinAngle;
      bool success = BiquadraticNumber::tryGetSine(angle, sinAngle);
      if (!success) { throw std::exception("Unsupported angle."); return polygon; }
      radius = half * radius / sinAngle;
    }
    for (int ii = 0; ii < nn; ++ii)
    {
      if (ii == 0)
      {
        Matrix<BiquadraticNumber> vec0;
        vec0.addRow({ radius, BiquadraticNumber() });
        vec0 = vec0.transpose();
        polygon.push_back(vec0);
        continue;
      }
      Rational angle(2 * ii, nn);
      BiquadraticNumber cosAngle, sinAngle;
      bool success = BiquadraticNumber::tryGetCosine(angle, cosAngle);
      success = success && BiquadraticNumber::tryGetSine(angle, sinAngle);
      if (!success) { throw std::exception("Unsupported angle."); break; }
      if ((generator != nullptr) && (ii == 1))
      {
        Matrix<BiquadraticNumber> R;
        R.addRow({ cosAngle, sinAngle });
        R.addRow({ -sinAngle, cosAngle });
        *generator = R;
      }
      Matrix<BiquadraticNumber> vec;
      vec.addRow({ cosAngle * radius, -sinAngle * radius });
      polygon.push_back(vec.transpose());
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

  /** Given vertices u and v having z-coordinate zero along with the origin w, and 
   *  proceeding clockwise along a pentagonal face of a regular dodecahedron,
   *  returns the unique vertex X in the dodecahedron such that |X - u| == |X - w| and |X - v| == |u - v|.
   *  Vertices are matrices with 3 rows and 1 column.
   *  Interchanging u and w does not change the result.
   */
  Matrix<BiquadraticNumber> completeEquilateralInDodeca_(const Matrix<BiquadraticNumber>& u,
    const Matrix<BiquadraticNumber>& v, bool usePlusSign)
  {
    Matrix<BiquadraticNumber> XX;
    if (u.numRows() != 3) { throw std::invalid_argument("Number of rows in u must == 3."); return XX; }
    if (v.numRows() != 3) { throw std::invalid_argument("Number of rows in v must == 3."); return XX; }
    if (u.numCols() != 1) { throw std::invalid_argument("Number of columns in u must == 1."); return XX; }
    if (v.numCols() != 1) { throw std::invalid_argument("Number of columns in v must == 1."); return XX; }
    if (u.at(2, 0) != BiquadraticNumber()) { throw std::invalid_argument("u.z must == 0."); return XX; }
    if (v.at(2, 0) != BiquadraticNumber()) { throw std::invalid_argument("v.z must == 0."); return XX; }
    if ((u - v).matrixSqNorm() != v.matrixSqNorm()) { throw std::invalid_argument("|u - v| must == |v|."); return XX; }
    BiquadraticNumber two(Rational(2));
    auto uSqNorm = u.matrixSqNorm();
    auto det = (u.at(0, 0) * v.at(1, 0) - u.at(1, 0) * v.at(0, 0));
    auto factor = uSqNorm / (two * det);

    auto xx = (v.at(1, 0) - u.at(1, 0)) * factor;
    auto yy = (u.at(0, 0) - v.at(0, 0)) * factor;

    auto radicand = uSqNorm - xx * xx - yy * yy;
    QuadraticNumber quad;
    if (!radicand.getAsQuadratic(quad))
    {
      throw std::exception("The z component is not the square root of a quadratic number.");
      Matrix<BiquadraticNumber> answer;
      return answer;
    }
    auto zz = BiquadraticNumber::sqrt(quad);
    if (!usePlusSign) { zz = -zz; }
    Matrix<BiquadraticNumber> answer;
    answer.addRow({ xx, yy, zz });
    answer = answer.transpose();
    auto answerSq = answer.matrixSqNorm();
    if (answerSq != uSqNorm) { throw std::invalid_argument("|u| must == |answer|."); }
    if ((u - answer).matrixSqNorm() != answerSq) { throw std::invalid_argument("|answer| must == |u - answer|."); }
    if ((v - answer).matrixSqNorm() != v.matrixSqNorm()) { throw std::invalid_argument("|v| must == |v - answer|."); }
    return answer;
  }

  /** Given vertices u, v, w proceeding clockwise along a pentagonal face of a regular dodecahedron,
   *  returns the unique vertex X in the dodecahedron such that |X - u| == |X - w| and |X - v| == |u - v|.
   *  Vertices are matrices with 3 rows and 1 column.
   *  Interchanging u and w does not change the result.
   *  \param `rotToZ_EqualsZero` is the rotation that maps the vertices to the plane { z == 0 }.
   *  \param `dodecIsNonnegative` is true if, and only if, the dodecahedron so far (which is convex and always lies
   *  on one side of the plane) lies on the nonnegative side.
   */
  Matrix<BiquadraticNumber> completeEquilateralInDodeca(const Matrix<BiquadraticNumber>& u,
    const Matrix<BiquadraticNumber>& v, const Matrix<BiquadraticNumber>& w,
    const Matrix<BiquadraticNumber>& rotToZ_EqualsZero, bool dodecIsNonnegative)
  {
    auto R_inv = rotToZ_EqualsZero.transpose();
    auto uu = rotToZ_EqualsZero * (u - w);
    auto vv = rotToZ_EqualsZero * (v - w);
    auto answer = completeEquilateralInDodeca_(uu, vv, dodecIsNonnegative);
    return (R_inv * answer) + w;
  }

  /** Given vertices u, v, w proceeding clockwise along a pentagonal face of a regular dodecahedron,
   *  adds the remaining two vertices to the dodecahedron if not already present.
   *  \throw Throws an exception if the dodecahedron doesn't have the length of its edges equal to `sideLength`
   */
  void completePentagonalFace(const Matrix<BiquadraticNumber>& u,
    const Matrix<BiquadraticNumber>& v, const Matrix<BiquadraticNumber>& w,
    const BiquadraticNumber& sideLength,
    std::set<Matrix<BiquadraticNumber> >& dodec)
  {
    bool dodecIsNonnegative = false;
    auto rot = getRotationToPlane(u, v, w, dodec, dodecIsNonnegative);
    auto uu = rot * (u - w);
    auto vv = rot * (v - w);
    auto sideLenSq = sideLength * sideLength;
    if ((uu - vv).matrixSqNorm() != sideLenSq)
    {
      throw std::invalid_argument("Improper side lengths for dodecahedron.");
    }
    if (vv.matrixSqNorm() != sideLenSq)
    {
      throw std::invalid_argument("Improper side lengths for dodecahedron.");
    }
    Matrix<BiquadraticNumber> planarRot5;
    std::vector<Matrix<BiquadraticNumber> > pentagon = getRegularPolygon(5, sideLength, &planarRot5);
    // TODO: Complete this later.
    auto rotInv = rot.transpose();
  }

  /** \brief Given vertices u, v, w proceeding clockwise along a pentagonal face of a regular dodecahedron,
   *  returns the rotation R that maps (u - w) and (v - w) to the plane { z == 0 }.
   *  \param Outputs dodecIsNonnegative if and only if the dodecahedron so far (which is convex and always lies
   *  on one side of the plane) lies on the nonnegative side.
   */
  Matrix<BiquadraticNumber> getRotationToPlane(const Matrix<BiquadraticNumber>& u,
    const Matrix<BiquadraticNumber>& v, const Matrix<BiquadraticNumber>& w,
    const std::set<Matrix<BiquadraticNumber> >& dodecSoFar, bool& dodecIsNonnegative)
  {
    BiquadraticNumber zero_(Rational(0));
    BiquadraticNumber one_(Rational(1));
    Matrix<BiquadraticNumber> II;
    II.addRow({ one_, zero_, zero_ });
    II.addRow({ zero_, one_, zero_ });
    II.addRow({ zero_, zero_, one_ });
    if ((u.at(2, 0) == w.at(2, 0)) && (v.at(2, 0) == w.at(2, 0)))
    {
      dodecIsNonnegative = true;
      for (const auto& vert : dodecSoFar)
      {
        if (vert.at(2, 0) > w.at(2, 0)) { break; }
        if (vert.at(2, 0) < w.at(2, 0)) { dodecIsNonnegative = false; break; }
      }
      return II;
    }
    Matrix<BiquadraticNumber> RR = II;
    BiquadraticNumber two(Rational(2));
    auto uu = u - w;
    auto vv = v - w;
    BiquadraticNumber crossProdNorm, uuNormTimesVvNorm, uvCrossNorm;
    {
      crossProdNorm = (uu.at(0, 0) * uu.at(0, 0)) * (vv.at(2, 0) * vv.at(2, 0));
      crossProdNorm = crossProdNorm + (uu.at(2, 0) * uu.at(2, 0)) * (vv.at(0, 0) * vv.at(0, 0));
      crossProdNorm = crossProdNorm + (vv.at(2, 0) * vv.at(2, 0)) * (uu.at(1, 0) * uu.at(1, 0));
      crossProdNorm = crossProdNorm + (uu.at(2, 0) * uu.at(2, 0)) * (vv.at(1, 0) * vv.at(1, 0));
      crossProdNorm = crossProdNorm - two * uu.at(0, 0) * uu.at(2, 0) * vv.at(0, 0) * vv.at(2, 0);
      crossProdNorm = crossProdNorm - two * uu.at(1, 0) * uu.at(2, 0) * vv.at(1, 0) * vv.at(2, 0);
      QuadraticNumber quad;
      bool success = crossProdNorm.getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("Cross product squared norm cannot be written in terms of quadratic numbers.");
        return RR;
      }
      crossProdNorm = BiquadraticNumber::sqrt(quad);
      uuNormTimesVvNorm = uu.matrixSqNorm() * vv.matrixSqNorm();
      success = uuNormTimesVvNorm.getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("|u - w|^2 * |v - w|^2 cannot be written in terms of quadratic numbers.");
        return RR;
      }
      uuNormTimesVvNorm = BiquadraticNumber::sqrt(quad);
      auto uvCross_0 = uu.at(1, 0) * vv.at(2, 0) - vv.at(1, 0) * uu.at(2, 0);
      auto uvCross_1 = uu.at(0, 0) * vv.at(2, 0) - vv.at(0, 0) * uu.at(2, 0);
      auto uvCross_2 = uu.at(0, 0) * vv.at(1, 0) - vv.at(0, 0) * uu.at(1, 0);
      success = (uvCross_0 * uvCross_0 + uvCross_1 * uvCross_1 + uvCross_2 * uvCross_2).getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("|(u - w) x (v - w)|^2 cannot be written in terms of quadratic numbers.");
        return RR;
      }
      uvCrossNorm = BiquadraticNumber::sqrt(quad);
    }
    Matrix<BiquadraticNumber> KK; // Cross product matrix.
    {
      auto k0 = vv.at(0, 0) * uu.at(2, 0) - uu.at(0, 0) * vv.at(2, 0);
      auto k1 = vv.at(1, 0) * uu.at(2, 0) - uu.at(1, 0) * vv.at(2, 0);
      KK.addRow({ zero_, zero_, k1 });
      KK.addRow({ zero_, zero_, -k0 });
      KK.addRow({ -k1, k0, zero_ });
    }
    RR = RR + KK * (uvCrossNorm / uuNormTimesVvNorm);
    RR = RR + (KK * KK) * (one_ - uu.matrixDot(vv) / uuNormTimesVvNorm);
    if (RR * RR.transpose() != II)
    {
      throw std::logic_error("Did not define a true rotation matrix.");
    }
    dodecIsNonnegative = true;
    auto R_ww = RR * w;
    for (const auto& vert : dodecSoFar)
    {
      auto R_vert = RR * vert;
      if (R_vert.at(2, 0) > R_ww.at(2, 0)) { break; }
      if (R_vert.at(2, 0) < R_ww.at(2, 0)) { dodecIsNonnegative = false; break; }
    }
    return RR;
  }

  std::set<Matrix<BiquadraticNumber> > getDodecahedron(const BiquadraticNumber& edgeLength)
  {
    std::set<Matrix<BiquadraticNumber> > dodec;
    BiquadraticNumber sideLength;
    {
      {
        Rational angle(1, 5);
        BiquadraticNumber half(Rational(1, 2));
        BiquadraticNumber sinAngle;
        bool success = BiquadraticNumber::tryGetSine(angle, sinAngle);
        if (!success) { throw std::exception("Unsupported angle."); return dodec; }
        sideLength = sinAngle + sinAngle;
      }
      // sideLength = edgeLength;
      auto initialPentagon = getRegularPolygon(5, sideLength);
      for (const auto& vertex : initialPentagon)
      {
        auto vert = vertex;
        vert.addRow({ BiquadraticNumber() });
        dodec.insert(vert);
      }
    }
    auto scaleFactor = edgeLength / sideLength;
    auto sideLengthSq = sideLength * sideLength;
    std::set<Matrix<BiquadraticNumber> > counted;
    for (int counting = 0; counting < 40; ++counting)
    {
      if (dodec.size() >= 20) { break; }
      bool found = false;
      Matrix<BiquadraticNumber> current = *(dodec.begin());
      std::vector<Matrix<BiquadraticNumber> > neighbors;
      for (const auto& vv : dodec)
      {
        if (counted.find(vv) != counted.end()) { continue; }
        neighbors.clear();
        for (const auto& ww : dodec)
        {
          if (vv == ww) { continue; }
          if (neighbors.size() >= 2)
          {
            counted.insert(vv);
            found = true;
            break;
          }
          if ((vv - ww).matrixSqNorm() == sideLengthSq)
          {
            neighbors.push_back(ww);
          }
        }
        if (neighbors.size() >= 2)
        {
          counted.insert(vv);
          found = true;
        }
        if (!found) { continue; }
        current = vv;
        break;
      }
      if (!found) { break; }
      auto oldSize = dodec.size();
      bool dodecIsNonnegative = false;
      auto rot = getRotationToPlane(neighbors[0], current, neighbors[1], dodec, dodecIsNonnegative);
      auto newVertex = completeEquilateralInDodeca(neighbors[0], current, neighbors[1], rot, dodecIsNonnegative);
      if (dodec.find(newVertex) != dodec.end())
      {
        found = false;
        auto oldCountedSize = counted.size();
        counted.insert(newVertex);
        if (oldCountedSize == counted.size()) { break; }
        continue;
      }
      dodec.insert(newVertex);
      completePentagonalFace(neighbors[0], current, newVertex, sideLength, dodec);
    }
    std::set<Matrix<BiquadraticNumber> > dodecahedron;
    {
      Matrix<BiquadraticNumber> barycenter = Matrix<BiquadraticNumber>::zeroMatrix(3, 1);
      const auto numVertices = (int)dodec.size();
      if (numVertices == 0) { return dodecahedron; }
      auto coeff = Matrix<BiquadraticNumber>({ Rational(1, numVertices) });
      for (const auto& vertex : dodec)
      {
        barycenter = barycenter + (vertex * coeff);
      }
      for (const auto& vertex : dodec)
      {
        dodecahedron.insert((vertex - barycenter) * scaleFactor);
      }
    }
    return dodecahedron;
  }
}
