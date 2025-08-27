/*  Copyright Paul Cernea, July 2025.
All Rights Reserved.*/

#include "platonic_solids.h"

#include <iostream>
#include <fstream>

namespace FunctionalCalculator
{
  struct CompareFaces
  {
    bool operator()(const std::vector<int>& lhs, const std::vector<int>& rhs) const
    {
      if (lhs.size() < rhs.size()) { return true; }
      if (lhs.size() > rhs.size()) { return false; }
      auto lhs_ = lhs; auto rhs_ = rhs;
      std::sort(lhs_.begin(), lhs_.end());
      std::sort(rhs_.begin(), rhs_.end());
      auto lhsSize = (int)lhs_.size();
      for (int ii = 0; ii < lhsSize; ++ii)
      {
        if (lhs_[ii] < rhs_[ii]) { return true; }
        if (lhs_[ii] > rhs_[ii]) { return false; }
      }
      return false;
    }
  };

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

  bool exportCubeObj(const std::string& filename)
  {
    BiquadraticNumber edgLength(Rational(1, 1));
    auto sqLen = edgLength * edgLength;
    std::map<int, Matrix<BiquadraticNumber> > cube;
    {
      auto cube0 = getCube(edgLength);
      int ii = -1;
      for (const auto& vertex : cube0)
      {
        ++ii;
        cube[ii] = vertex.transpose();
      }
    }
    std::set<std::vector<int>, CompareFaces> faces;
    faces.insert({ 0, 1, 3, 2 });
    faces.insert({ 0, 2, 6, 4 });
    faces.insert({ 7, 6, 2, 3 });
    faces.insert({ 1, 0, 4, 5 });
    faces.insert({ 6, 7, 5, 4 });
    faces.insert({ 7, 3, 1, 5 });

    std::ofstream obj(filename);
    if (!(obj.good())) { return false; }
    obj << "\n";

    bool success = true;
    try
    {
      for (const auto& iter : cube)
      {
        obj << "\nv " << iter.second.at(0, 0).get().first;
        obj << " " << iter.second.at(1, 0).get().first;
        obj << " " << iter.second.at(2, 0).get().first;
      }
      obj << "\n";
      for (const auto& face : faces)
      {
        obj << "\nf";
        for (const auto& vert : face)
        {
          obj << " " << (vert + 1);
        }
      }
    }
    catch (...)
    {
      success = false;
    }
    return success;
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
    Matrix<BiquadraticNumber> uuCrossVv;
    uuCrossVv.addRow({ uu.at(1, 0) * vv.at(2, 0) - vv.at(1, 0) * uu.at(2, 0),
                       uu.at(2, 0) * vv.at(0, 0) - vv.at(2, 0) * uu.at(0, 0),
                       uu.at(0, 0) * vv.at(1, 0) - vv.at(0, 0) * uu.at(1, 0) });
    uuCrossVv = uuCrossVv.transpose();
    Matrix<BiquadraticNumber> kVec; // == (uu x vv) x (0, 0, 1)
    kVec.addRow({ uuCrossVv.at(1, 0), -uuCrossVv.at(0, 0), BiquadraticNumber() });
    kVec = kVec.transpose();
    BiquadraticNumber kNorm, uuNormTimesVvNorm, uuCrossVvNorm;
    {
      kNorm = kVec.at(0, 0) * kVec.at(0, 0) + kVec.at(1, 0) * kVec.at(1, 0);
      uuCrossVvNorm = kNorm + uuCrossVv.at(2, 0) * uuCrossVv.at(2, 0);
      QuadraticNumber quad;
      bool success = kNorm.getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("|((u - w) x (v - w)) x (0, 0, 1)|^2 cannot be written in terms of quadratic numbers.");
        return RR;
      }
      kNorm = BiquadraticNumber::sqrt(quad);
      success = uuCrossVvNorm.getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("|(u - w) x (v - w)|^2 cannot be written in terms of quadratic numbers.");
        return RR;
      }
      uuCrossVvNorm = BiquadraticNumber::sqrt(quad);
      uuNormTimesVvNorm = uu.matrixSqNorm() * vv.matrixSqNorm();
      success = uuNormTimesVvNorm.getAsQuadratic(quad);
      if (!success)
      {
        throw std::exception("|u - w|^2 * |v - w|^2 cannot be written in terms of quadratic numbers.");
        return RR;
      }
      uuNormTimesVvNorm = BiquadraticNumber::sqrt(quad);
    }
    kVec = kVec * (one_ / kNorm);
    if (kVec.matrixSqNorm() != one_)
    {
      throw std::logic_error("Bad unit vector computation.");
    }
    if (kVec.matrixDot(uuCrossVv) != zero_)
    {
      throw std::logic_error("Bad orthogonality computation.");
    }
    if (kVec.at(2, 0) != zero_)
    {
      throw std::logic_error("Bad orthogonality computation.");
    }
    Matrix<BiquadraticNumber> KK; // Matrix that maps X to kVec cross X.
    KK.addRow({ zero_, -kVec.at(2, 0), kVec.at(1, 0) });
    KK.addRow({ kVec.at(2, 0), zero_, -kVec.at(0, 0) });
    KK.addRow({ -kVec.at(1, 0), kVec.at(0, 0), zero_ });

    auto absSinTheta = kNorm / uuCrossVvNorm;
    auto cosTheta = uuCrossVv.at(2, 0) / uuCrossVvNorm;
    if ((absSinTheta * absSinTheta + cosTheta * cosTheta) != one_)
    {
      throw std::logic_error("Bad trigonometric computation.");
    }
    RR = II;
    RR = RR + KK * absSinTheta;
    RR = RR + (KK * KK) * (one_ - cosTheta);
    auto rotCrossProduct = RR * uuCrossVv;
    if (RR * RR.transpose() != II)
    {
      RR = II;
      RR = RR - KK * absSinTheta;
      RR = RR + (KK * KK) * (one_ - cosTheta);
      rotCrossProduct = RR * uuCrossVv;
    }
    else if ((rotCrossProduct.at(0, 0) != zero_) || (rotCrossProduct.at(1, 0) != zero_))
    {
      RR = II;
      RR = RR - KK * absSinTheta;
      RR = RR + (KK * KK) * (one_ - cosTheta);
      rotCrossProduct = RR * uuCrossVv;
    }
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
    if ((rotCrossProduct.at(0, 0) != zero_) || (rotCrossProduct.at(1, 0) != zero_))
    {
      throw std::logic_error("(u - w) and (v - w) did not get mapped to { z == 0 } by the rotation.");
    }
    return RR;
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
    if ((pentagon[1] - pentagon[0]).matrixSqNorm() != sideLenSq)
    {
      throw std::invalid_argument("Improper side lengths for dodecahedron.");
    }
    if ((pentagon[2] - pentagon[1]).matrixSqNorm() != sideLenSq)
    {
      throw std::invalid_argument("Improper side lengths for dodecahedron.");
    }
    if ((pentagon[2] - pentagon[0]).matrixSqNorm() != uu.matrixSqNorm())
    {
      throw std::invalid_argument("Improper side lengths for dodecahedron.");
    }
    if ((pentagon[0] - pentagon[2]).matrixDot(pentagon[1] - pentagon[2]) != uu.matrixDot(vv))
    {
      throw std::invalid_argument("Improper angles for dodecahedron.");
    }

    // Unique matrix P such that P^{-1} * x + p[2] takes (uu, vv, 0) to (p[0], p[1], p[2]) where p is vector `pentagon`.
    Matrix<BiquadraticNumber> isometry;
    {
      BiquadraticNumber zero_(Rational(0));
      BiquadraticNumber one_(Rational(1));
      {
        const auto& p = pentagon;
        Matrix<BiquadraticNumber> uvCols;
        uvCols.addRow({ uu.at(0, 0), vv.at(0, 0) });
        uvCols.addRow({ uu.at(1, 0), vv.at(1, 0) });
        Matrix<BiquadraticNumber> pentaColsInv;
        auto aa = p[0].at(0, 0) - p[2].at(0, 0);
        auto bb = p[1].at(0, 0) - p[2].at(0, 0);
        auto cc = p[0].at(1, 0) - p[2].at(1, 0);
        auto dd = p[1].at(1, 0) - p[2].at(1, 0);
        auto determ = aa * dd - bb * cc;
        auto factor = one_ / determ;
        pentaColsInv.addRow({ dd * factor, -bb * factor });
        pentaColsInv.addRow({ -cc * factor, aa * factor });
        isometry = uvCols * pentaColsInv;
      }
      Matrix<BiquadraticNumber> II;
      II.addRow({ one_, zero_ });
      II.addRow({ zero_, one_ });
      auto identity = isometry * (isometry.transpose());
      if (identity != II)
      {
        throw std::logic_error("Pentagonal isometry did not define a true planar rotation matrix.");
      }
    }
    auto vv3 = isometry * (pentagon[3] - pentagon[2]);
    auto vv4 = isometry * (pentagon[4] - pentagon[2]);
    vv3.addRow({ BiquadraticNumber() });
    vv4.addRow({ BiquadraticNumber() });

    auto rotInv = rot.transpose();
    auto v3 = rotInv * vv3 + w;
    auto v4 = rotInv * vv4 + w;
    dodec.insert(v3);
    dodec.insert(v4);
  }

  std::set<Matrix<BiquadraticNumber> > recenterToOriginAndScale(const std::set<Matrix<BiquadraticNumber> >& shape,
    const BiquadraticNumber& scaleFactor)
  {
    std::set<Matrix<BiquadraticNumber> > answer;
    Matrix<BiquadraticNumber> barycenter = Matrix<BiquadraticNumber>::zeroMatrix(3, 1);
    const auto numVertices = (int)shape.size();
    if (numVertices == 0) { return answer; }
    auto coeff = Matrix<BiquadraticNumber>({ Rational(1, numVertices) });
    for (const auto& vertex : shape)
    {
      barycenter = barycenter + (vertex * coeff);
    }
    for (const auto& vertex : shape)
    {
      answer.insert((vertex - barycenter) * scaleFactor);
    }
    return answer;
  }

  std::set<Matrix<BiquadraticNumber> > growDodecahedron(const BiquadraticNumber& edgeLength)
  {
    BiquadraticNumber::setExtraSimplification(true);
    std::set<Matrix<BiquadraticNumber> > dodec;
    BiquadraticNumber sideLength;
    {
      {
        Rational angle(1, 5);
        BiquadraticNumber half(Rational(1, 2));
        BiquadraticNumber sinAngle;
        bool success = BiquadraticNumber::tryGetSine(angle, sinAngle);
        if (!success)
        {
          BiquadraticNumber::setExtraSimplification(false);
          throw std::exception("Unsupported angle.");
          return dodec;
        }
        sideLength = sinAngle + sinAngle;
      }
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
    auto oldCountedSize = counted.size();
    auto oldDodecSize = dodec.size();
    for (bool started = false; true; started = true)
    {
      if (started)
      {
        if ((oldCountedSize == counted.size()) &&
          (oldDodecSize == dodec.size())) {
          break;
        }
        oldCountedSize = counted.size();
        oldDodecSize = dodec.size();
      }
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

    auto dodecahedron = recenterToOriginAndScale(dodec, scaleFactor);
    BiquadraticNumber::setExtraSimplification(false);
    return dodecahedron;
  }

  std::set<Matrix<BiquadraticNumber> > getDodecahedron(const BiquadraticNumber& edgeLength)
  {
    BiquadraticNumber::setExtraSimplification(true);
    std::set<Matrix<BiquadraticNumber> > dodec;
    BiquadraticNumber sideLength;
    {
      Rational angle(1, 5);
      BiquadraticNumber half(Rational(1, 2));
      BiquadraticNumber sinAngle;
      bool success = BiquadraticNumber::tryGetSine(angle, sinAngle);
      if (!success)
      {
        BiquadraticNumber::setExtraSimplification(false);
        throw std::exception("Unsupported angle.");
        return dodec;
      }
      sideLength = sinAngle + sinAngle;
    }
    auto zero_ = BiquadraticNumber();
    BiquadraticNumber one_(Rational(1));
    auto scaleFactor = edgeLength / sideLength;
    BiquadraticNumber half(Rational(1, 2));
    BiquadraticNumber threeHalves(Rational(3, 2));
    BiquadraticNumber quarter(Rational(1, 4));
    BiquadraticNumber threeFourths(Rational(3, 4));
    BiquadraticNumber s1 = BiquadraticNumber::sqrt(QuadraticNumber(Rational(5, 8))
      + QuadraticNumber(Rational(1, 8)) * QuadraticNumber::sqrt(5));
    BiquadraticNumber s2 = BiquadraticNumber::sqrt(QuadraticNumber(Rational(5, 8))
      - QuadraticNumber(Rational(1, 8)) * QuadraticNumber::sqrt(5));
    BiquadraticNumber s3 = BiquadraticNumber::sqrt(QuadraticNumber(Rational(5, 4))
      + QuadraticNumber(Rational(1, 2)) * QuadraticNumber::sqrt(5));
    auto sqrt5 = BiquadraticNumber::sqrt(Rational(5));
    auto golden = half + half * sqrt5;
    {
      // 0: (-1 / 2 - (1 / 2) * Sqrt(5), 0, 1 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -golden, zero_, golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 1: (-3 / 4 - (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -threeFourths - quarter * sqrt5, -s1, one_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 2: (-3 / 4 - (1 / 4) * Sqrt(5), Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -threeFourths - quarter * sqrt5, s1, one_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 3: (-1, 0, 3 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -one_, zero_, threeHalves + half * sqrt5 });
      dodec.insert(vertex.transpose());
    }
    {
      // 4: (-1 / 4 - (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 - (1 / 8) * Sqrt(5)), 0)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -quarter - quarter * sqrt5, -s2, zero_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 5: (-1 / 4 - (1 / 4) * Sqrt(5), Sqrt(5 / 8 - (1 / 8) * Sqrt(5)), 0)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -quarter - quarter * sqrt5, s2, zero_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 6: (-1 / 2, (-1) * Sqrt(5 / 4 + (1 / 2) * Sqrt(5)), 1 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half, -s3, golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 7: (-1 / 2, Sqrt(5 / 4 + (1 / 2) * Sqrt(5)), 1 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half, s3, golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 8: (1 / 4 - (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 3 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ quarter - quarter * sqrt5, -s1, one_ + golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 9: (1 / 4 - (1 / 4) * Sqrt(5), Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 3 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ quarter - quarter * sqrt5, s1, one_ + golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 10: (-1 / 4 + (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 0)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -quarter + quarter * sqrt5, -s1, zero_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 11: (-1 / 4 + (1 / 4) * Sqrt(5), Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 0)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -quarter + quarter * sqrt5, s1, zero_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 12: (1 / 2, (-1) * Sqrt(5 / 4 + (1 / 2) * Sqrt(5)), 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half, -s3, one_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 13: (1 / 2, Sqrt(5 / 4 + (1 / 2) * Sqrt(5)), 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half, s3, one_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 14: (1 / 4 + (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 - (1 / 8) * Sqrt(5)), 3 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half * golden, -s2, one_ + golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 15: (1 / 4 + (1 / 4) * Sqrt(5), Sqrt(5 / 8 - (1 / 8) * Sqrt(5)), 3 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half * golden, s2, one_ + golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 16: (1, 0, 0)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ one_, zero_, zero_ });
      dodec.insert(vertex.transpose());
    }
    {
      // 17: (3 / 4 + (1 / 4) * Sqrt(5), (-1) * Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 1 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ threeFourths + quarter * sqrt5, -s1, golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 18: (3 / 4 + (1 / 4) * Sqrt(5), Sqrt(5 / 8 + (1 / 8) * Sqrt(5)), 1 / 2 + (1 / 2) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ threeFourths + quarter * sqrt5, s1, golden });
      dodec.insert(vertex.transpose());
    }
    {
      // 19: (1 / 2 + (1 / 2) * Sqrt(5), 0, 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ golden, zero_, one_ });
      dodec.insert(vertex.transpose());
    }
    auto dodecahedron = recenterToOriginAndScale(dodec, scaleFactor);
    BiquadraticNumber::setExtraSimplification(false);
    return dodecahedron;
  }

  bool exportDodecahedronObj(const std::string& filename)
  {
    BiquadraticNumber edgeLengthSq(Rational(1, 1));
    int facetSize = 5;
    int expectedNumVertices = 20;
    int expectedNumNeighbors = 3;
    int expectedNumFaces = 12;
    std::map<int, Matrix<BiquadraticNumber> > dodec;
    {
      int ii = -1;
      auto dodecSet = getDodecahedron();
      for (const auto& vertex : dodecSet)
      {
        ++ii;
        dodec[ii] = vertex;
      }
    }
    if ((int)dodec.size() != expectedNumVertices) { return false; }

    std::map<int, std::vector<int> > adjacencyGraph;
    for (const auto& iter : dodec)
    {
      std::vector<int> neighbors;
      for (const auto& jter : dodec)
      {
        auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
        if (neighborSqDist == edgeLengthSq)
        {
          neighbors.push_back(jter.first);
        }
      }
      if ((int)neighbors.size() != expectedNumNeighbors) { return false; }
      adjacencyGraph[iter.first] = neighbors;
    }

    std::set<std::vector<int>, CompareFaces> faces;
    for (const auto& iter : adjacencyGraph)
    {
      std::vector<std::vector<int> > facesFrom;
      facesFrom.push_back({ iter.second[0], iter.first, iter.second[1] });
      facesFrom.push_back({ iter.second[1], iter.first, iter.second[2] });
      facesFrom.push_back({ iter.second[2], iter.first, iter.second[0] });
      auto verticesRemain = (int)(facetSize - facesFrom[0].size());
      for (const auto& faceFrom : facesFrom)
      {
        auto current = faceFrom;
        for (int verticesRemaining = 0; verticesRemaining < verticesRemain; ++verticesRemaining)
        {
          int nextOne = -1;
          for (const auto& subsequent : adjacencyGraph[current[current.size() - 1]])
          {
            bool doNotAdd = false;
            for (int ii = 0; ii < (int)current.size(); ++ii)
            {
              if (subsequent != current[ii]) { continue; }
              doNotAdd = true; break;
            }
            if (doNotAdd) { continue; }
            if (nextOne == -1) { nextOne = subsequent; continue; }
            if ((dodec[current[0]] - dodec[subsequent]).matrixSqNorm()
              < (dodec[current[0]] - dodec[nextOne]).matrixSqNorm())
            {
              nextOne = subsequent; continue;
            }
          }
          current.push_back(nextOne);
        }
        faces.insert(current);
      }
    }
    if ((int)faces.size() != expectedNumFaces) { return false; }

    std::ofstream obj(filename);
    if (!(obj.good())) { return false; }
    obj << "\n";

    bool success = true;
    try
    {
      for (const auto& iter : dodec)
      {
        obj << "\nv " << iter.second.at(0, 0).get().first;
        obj << " " << iter.second.at(1, 0).get().first;
        obj << " " << iter.second.at(2, 0).get().first;
      }
      obj << "\n";
      for (const auto& face : faces)
      {
        obj << "\nf";
        for (const auto& vert : face)
        {
          obj << " " << (vert + 1);
        }
      }
    }
    catch (...)
    {
      success = false;
    }
    return success;
  }

  std::set<Matrix<BiquadraticNumber> > getIcosahedronViaDual(const BiquadraticNumber& edgeLength)
  {
    std::set<Matrix<BiquadraticNumber> > icosa;
    auto defaultRadius = QuadraticNumber(Rational(9, 8)) + QuadraticNumber::sqrt(Rational(45, 64));
    //auto dodecEdgeLength = BiquadraticNumber(Rational(1, 1)) / BiquadraticNumber::sqrt(defaultRadius);
    auto dodecEdgeLength = BiquadraticNumber::sqrt(QuadraticNumber(Rational(5)) + QuadraticNumber::sqrt(Rational(20)));
    BiquadraticNumber dodecEdgeLengthSq = dodecEdgeLength * dodecEdgeLength;
    int dodecFaceSize = 5;
    int dodecNumVertices = 20;
    int dodecVtxNumNeighbors = 3;
    int dodecNumFaces = 12;
    // A dodecahedron dual to the icosahedron:
    std::map<int, Matrix<BiquadraticNumber> > dualDodec;
    {
      int ii = -1;
      auto dodecSet = getDodecahedron(dodecEdgeLength);
      for (const auto& vertex : dodecSet)
      {
        ++ii;
        dualDodec[ii] = vertex;
      }
    }
    if ((int)dualDodec.size() != dodecNumVertices)
    {
      throw std::logic_error("Dual dodecahedron should have 20 vertices.");
      return icosa;
    }

    std::map<int, std::vector<int> > adjacencyGraph;
    for (const auto& iter : dualDodec)
    {
      std::vector<int> neighbors;
      for (const auto& jter : dualDodec)
      {
        auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
        if (neighborSqDist == dodecEdgeLengthSq)
        {
          neighbors.push_back(jter.first);
        }
      }
      if ((int)neighbors.size() != dodecVtxNumNeighbors)
      {
        throw std::logic_error("Dual dodecahedron vertices should each have 3 neighbors.");
        return icosa;
      }
      adjacencyGraph[iter.first] = neighbors;
    }

    std::set<std::vector<int>, CompareFaces> faces;
    for (const auto& iter : adjacencyGraph)
    {
      std::vector<std::vector<int> > facesFrom;
      facesFrom.push_back({ iter.second[0], iter.first, iter.second[1] });
      facesFrom.push_back({ iter.second[1], iter.first, iter.second[2] });
      facesFrom.push_back({ iter.second[2], iter.first, iter.second[0] });
      auto verticesRemain = (int)(dodecFaceSize - facesFrom[0].size());
      for (const auto& faceFrom : facesFrom)
      {
        auto current = faceFrom;
        for (int verticesRemaining = 0; verticesRemaining < verticesRemain; ++verticesRemaining)
        {
          int nextOne = -1;
          for (const auto& subsequent : adjacencyGraph[current[current.size() - 1]])
          {
            bool doNotAdd = false;
            for (int ii = 0; ii < (int)current.size(); ++ii)
            {
              if (subsequent != current[ii]) { continue; }
              doNotAdd = true; break;
            }
            if (doNotAdd) { continue; }
            if (nextOne == -1) { nextOne = subsequent; continue; }
            if ((dualDodec[current[0]] - dualDodec[subsequent]).matrixSqNorm()
              < (dualDodec[current[0]] - dualDodec[nextOne]).matrixSqNorm())
            {
              nextOne = subsequent; continue;
            }
          }
          current.push_back(nextOne);
        }
        faces.insert(current);
      }
    }
    if ((int)faces.size() != dodecNumFaces)
    {
      throw std::logic_error("Dual dodecahedron should have 12 faces.");
      return icosa;
    }
    for (const auto& face : faces)
    {
      BiquadraticNumber factor(Rational(1, dodecFaceSize));
      auto barycenter = Matrix<BiquadraticNumber>::zeroMatrix(3, 1);
      for (const auto& vtx : face)
      {
        barycenter = barycenter + dualDodec[vtx] * factor;
      }
      icosa.insert(barycenter);
    }
    std::set<Matrix<BiquadraticNumber> > icosaOut;

    QuadraticNumber two(Rational(2));
    QuadraticNumber five(Rational(5));
    auto sqrt5 = QuadraticNumber::sqrt(5);
    auto factor = edgeLength * BiquadraticNumber::sqrt(five + two * sqrt5) /
      (BiquadraticNumber(QuadraticNumber(Rational(7, 4)) + QuadraticNumber(Rational(3, 4)) * sqrt5));
    for (const auto& vv : icosa)
    {
      icosaOut.insert(vv * factor);
    }
    return icosaOut;
  }

  std::set<Matrix<BiquadraticNumber> > getIcosahedron(const BiquadraticNumber& edgeLength)
  {
    std::set<Matrix<BiquadraticNumber> > icosa;

    auto zero_ = BiquadraticNumber();
    BiquadraticNumber one_(Rational(1));
    BiquadraticNumber sideLength = one_;
    auto scaleFactor = edgeLength / sideLength;
    BiquadraticNumber half(Rational(1, 2));
    BiquadraticNumber threeHalves(Rational(3, 2));
    BiquadraticNumber fifth(Rational(1, 5));
    BiquadraticNumber tenth(Rational(1, 10));
    BiquadraticNumber twoFifths(Rational(2, 5));
    QuadraticNumber q_1(Rational(1));
    QuadraticNumber q_1_2(Rational(1, 2));
    QuadraticNumber q_2_5(Rational(2, 5));
    QuadraticNumber q_1_10(Rational(1, 10));
    auto qSqrt5 = QuadraticNumber::sqrt(5);
    auto sqrt5 = BiquadraticNumber::sqrt(5);
    BiquadraticNumber sPlus = BiquadraticNumber::sqrt(q_1_2 + q_1_10 * qSqrt5);
    BiquadraticNumber sMinus = BiquadraticNumber::sqrt(q_1_2 - q_1_10 * qSqrt5);

    {
      // 0: ((-2 / 5) * Sqrt(5), 0, (-1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -twoFifths * sqrt5, zero_, -fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 1: (-1 / 2 - (1 / 10) * Sqrt(5), (-1) * Sqrt(1 / 2 - (1 / 10) * Sqrt(5)), (1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half - tenth * sqrt5, -sMinus, fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 2: (-1 / 2 - (1 / 10) * Sqrt(5), Sqrt(1 / 2 - (1 / 10) * Sqrt(5)), (1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half - tenth * sqrt5, sMinus, fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 3: (-1 / 2 + (1 / 10) * Sqrt(5), (-1) * Sqrt(1 / 2 + (1 / 10) * Sqrt(5)), (-1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half + tenth * sqrt5, -sPlus, -fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 4: (-1 / 2 + (1 / 10) * Sqrt(5), Sqrt(1 / 2 + (1 / 10) * Sqrt(5)), (-1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ -half + tenth * sqrt5, sPlus, -fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 5: (0, 0, (-1) * Sqrt(1 / 5 + (2 / 25) * Sqrt(5)) - Sqrt(16 / 5 + (32 / 25) * Sqrt(5)))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ zero_, zero_, -one_ });
      icosa.insert(vertex.transpose());
    }
    {
      // 6: (1 / 2 - (1 / 10) * Sqrt(5), (-1) * Sqrt(1 / 2 + (1 / 10) * Sqrt(5)), (1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half - tenth * sqrt5, -sPlus, fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 7: (1 / 2 - (1 / 10) * Sqrt(5), Sqrt(1 / 2 + (1 / 10) * Sqrt(5)), (1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half - tenth * sqrt5, sPlus, fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 8: (1 / 2 + (1 / 10) * Sqrt(5), (-1) * Sqrt(1 / 2 - (1 / 10) * Sqrt(5)), (-1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half + tenth * sqrt5, -sMinus, -fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 9: (1 / 2 + (1 / 10) * Sqrt(5), Sqrt(1 / 2 - (1 / 10) * Sqrt(5)), (-1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ half + tenth * sqrt5, sMinus, -fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 10: ((2 / 5) * Sqrt(5), 0, (1 / 5) * Sqrt(5))
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ twoFifths * sqrt5, zero_, fifth * sqrt5 });
      icosa.insert(vertex.transpose());
    }
    {
      // 11: (, 0, 1)
      Matrix<BiquadraticNumber> vertex;
      vertex.addRow({ zero_, zero_, one_ });
      icosa.insert(vertex.transpose());
    }

    // edge lengths if vertices are unit length:
    auto dist = BiquadraticNumber::sqrt(q_1 + q_1 - q_2_5 * qSqrt5);
    auto factor = edgeLength / dist;
    std::set<Matrix<BiquadraticNumber> > icosaOut;
    for (const auto& vv : icosa)
    {
      icosaOut.insert(vv * factor);
    }

    return icosaOut;
  }

  std::set<Matrix<BiquadraticNumber> > getIcosahedralSymmetries(bool includeReflections)
  {
    BiquadraticNumber::setExtraSimplification(true);
    std::set<Matrix<BiquadraticNumber> > icosahedralSymmetries;
    static std::map<int, Matrix<BiquadraticNumber> > sDodecahedron;
    static std::set<std::vector<int>, CompareFaces> sDodecFaces;
    if (sDodecahedron.empty() || sDodecFaces.empty())
    {
      BiquadraticNumber edgeLengthSq(Rational(1, 1));
      int facetSize = 5;
      int expectedNumVertices = 20;
      int expectedNumNeighbors = 3;
      int expectedNumFaces = 12;
      std::map<int, Matrix<BiquadraticNumber> > dodec;
      {
        int ii = -1;
        auto dodecSet = getDodecahedron();
        for (const auto& vertex : dodecSet)
        {
          ++ii;
          dodec[ii] = vertex;
        }
      }
      if ((int)dodec.size() != expectedNumVertices)
      {
        BiquadraticNumber::setExtraSimplification(false);
        return icosahedralSymmetries;
      }

      std::map<int, std::vector<int> > adjacencyGraph;
      for (const auto& iter : dodec)
      {
        std::vector<int> neighbors;
        for (const auto& jter : dodec)
        {
          auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
          if (neighborSqDist == edgeLengthSq)
          {
            neighbors.push_back(jter.first);
          }
        }
        if ((int)neighbors.size() != expectedNumNeighbors)
        {
          BiquadraticNumber::setExtraSimplification(false);
          return icosahedralSymmetries;
        }
        adjacencyGraph[iter.first] = neighbors;
      }

      std::set<std::vector<int>, CompareFaces> faces;
      for (const auto& iter : adjacencyGraph)
      {
        std::vector<std::vector<int> > facesFrom;
        facesFrom.push_back({ iter.second[0], iter.first, iter.second[1] });
        facesFrom.push_back({ iter.second[1], iter.first, iter.second[2] });
        facesFrom.push_back({ iter.second[2], iter.first, iter.second[0] });
        auto verticesRemain = (int)(facetSize - facesFrom[0].size());
        for (const auto& faceFrom : facesFrom)
        {
          auto current = faceFrom;
          for (int verticesRemaining = 0; verticesRemaining < verticesRemain; ++verticesRemaining)
          {
            int nextOne = -1;
            for (const auto& subsequent : adjacencyGraph[current[current.size() - 1]])
            {
              bool doNotAdd = false;
              for (int ii = 0; ii < (int)current.size(); ++ii)
              {
                if (subsequent != current[ii]) { continue; }
                doNotAdd = true; break;
              }
              if (doNotAdd) { continue; }
              if (nextOne == -1) { nextOne = subsequent; continue; }
              if ((dodec[current[0]] - dodec[subsequent]).matrixSqNorm()
                < (dodec[current[0]] - dodec[nextOne]).matrixSqNorm())
              {
                nextOne = subsequent; continue;
              }
            }
            current.push_back(nextOne);
          }
          faces.insert(current);
        }
      }
      if ((int)faces.size() != expectedNumFaces)
      {
        BiquadraticNumber::setExtraSimplification(false);
        return icosahedralSymmetries;
      }
      sDodecahedron = dodec;
      sDodecFaces = faces;
    }
    QuadraticNumber q_1(Rational(1));
    for (const auto& face : sDodecFaces)
    {
      if (face.size() < 5) { throw std::logic_error("Dodecahedral faces should be pentagons."); }
      auto& uu = sDodecahedron[face[0]];
      auto& vv = sDodecahedron[face[1]];
      auto& ww = sDodecahedron[face[2]];
      auto UU = uu - ww;
      auto VV = vv - ww;
      auto factor = UU.matrixSqNorm();
      QuadraticNumber quad;
      if (!factor.getAsQuadratic(quad)) { throw std::invalid_argument("Dodecahedron vertex should square to quadratic number."); }
      factor = BiquadraticNumber::sqrt(q_1 / quad);
      UU = UU * factor;
      factor = VV.matrixSqNorm();
      if (!factor.getAsQuadratic(quad)) { throw std::invalid_argument("Dodecahedron vertex should square to quadratic number."); }
      factor = BiquadraticNumber::sqrt(q_1 / quad);
      VV = VV * factor;
      auto R_0 = Matrix<BiquadraticNumber>::getRotation(UU, VV);
      auto R = R_0;
      for (int ii = 0; ii < 5; ++ii)
      {
        icosahedralSymmetries.insert(R);
        R = R * R_0;
      }
    }
    Matrix<BiquadraticNumber> A;
    if (includeReflections)
    {
      A.addRow({ Rational(-1), Rational(0), Rational(0) });
      A.addRow({ Rational(0), Rational(-1), Rational(0) });
      A.addRow({ Rational(0), Rational(0) , Rational(-1) });
      icosahedralSymmetries.insert(A);
    }
    //int limit = includeReflections ? 2 * num : num;
    //while (icosahedralSymmetries.size() < limit)
    auto oldSize = icosahedralSymmetries.size();
    bool started = false;
    while (!started || (icosahedralSymmetries.size() != oldSize))
    {
      started = true;
      oldSize = icosahedralSymmetries.size();
      auto others = icosahedralSymmetries;
      for (const auto& rot : icosahedralSymmetries)
      {
        for (const auto& other : icosahedralSymmetries)
        {
          others.insert(rot * other);
        }
        if (includeReflections)
        {
          others.insert(A * rot);
        }
      }
      icosahedralSymmetries = others;
    }
    BiquadraticNumber::setExtraSimplification(false);
    return icosahedralSymmetries;
  }

  bool exportIcosahedronObj(const std::string& filename)
  {
    BiquadraticNumber edgeLengthSq(Rational(1, 1));
    int facetSize = 3;
    int expectedNumVertices = 12;
    int expectedNumEdges = 30;
    int expectedNumNeighbors = 5;
    int expectedNumFaces = 20;
    std::map<int, Matrix<BiquadraticNumber> > icosa;
    {
      int ii = -1;
      auto icosaSet = getIcosahedron();
      for (const auto& vertex : icosaSet)
      {
        ++ii;
        icosa[ii] = vertex;
      }
    }
    if ((int)icosa.size() != expectedNumVertices) { return false; }

    std::map<int, std::vector<int> > adjacencyGraph;
    for (const auto& iter : icosa)
    {
      std::vector<int> neighbors;
      for (const auto& jter : icosa)
      {
        auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
        if (neighborSqDist == edgeLengthSq)
        {
          neighbors.push_back(jter.first);
        }
      }
      if ((int)neighbors.size() != expectedNumNeighbors) { return false; }
      adjacencyGraph[iter.first] = neighbors;
    }

    std::set<std::pair<int, int> > halfEdges;
    for (const auto& iter : adjacencyGraph)
    {
      for (const auto& index : iter.second) { halfEdges.insert({ iter.first, index }); }
    }
    if ((int)halfEdges.size() != expectedNumEdges * 2) { return false; }

    std::set<std::vector<int>, CompareFaces> faces;
    for (const auto& iter : adjacencyGraph)
    {
      for (int ii = 0; ii < expectedNumNeighbors; ++ii)
      {
        int jj = (ii == expectedNumNeighbors - 1) ? 0 : (ii + 1);
        auto aa = iter.second[ii];
        auto bb = iter.second[jj];
        if ((halfEdges.find({ aa, bb }) == halfEdges.end()) && (halfEdges.find({ bb, aa }) == halfEdges.end()))
        {
          continue;
        }
        faces.insert({ iter.second[ii], iter.first, iter.second[jj] });
      }
    }
    if ((int)faces.size() != expectedNumFaces) { return false; }

    std::ofstream obj(filename);
    if (!(obj.good())) { return false; }
    obj << "\n";

    bool success = true;
    try
    {
      for (const auto& iter : icosa)
      {
        obj << "\nv " << iter.second.at(0, 0).get().first;
        obj << " " << iter.second.at(1, 0).get().first;
        obj << " " << iter.second.at(2, 0).get().first;
      }
      obj << "\n";
      for (const auto& face : faces)
      {
        obj << "\nf";
        for (const auto& vert : face)
        {
          obj << " " << (vert + 1);
        }
      }
    }
    catch (...)
    {
      success = false;
    }
    return success;
  }

  bool test_dodecahedron()
  {
    std::string prompt;
    BiquadraticNumber edgeLength(Rational(1, 1));
    auto edgeLengthSq = edgeLength * edgeLength;

    std::cout << "\nDODECAHEDRON centered at (0, 0, 0) with all edge lengths == " << edgeLength.print() << ":";
    std::map<int, Matrix<BiquadraticNumber> > dodec;
    {
      int ii = -1;
      // Grows the dodecahedron "organically" with minimal "understanding" and no hardcoded values but is slow.
      //auto dodecSet = growDodecahedron(edgeLength);

      auto dodecSet = getDodecahedron(edgeLength); // Uses cached values from a call to growDodecahedron().
      for (const auto& vertex : dodecSet)
      {
        ++ii;
        std::cout << "\nVertex " << ii << " = " << vertex.transpose().print(true);
        dodec[ii] = vertex;
      }
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    for (const auto& iter : dodec)
    {
      std::cout << "\nVertex " << iter.first << " sq. length = " << iter.second.matrixSqNorm().print();
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::map<int, std::vector<int> > adjacencyGraph;

    for (const auto& iter : dodec)
    {
      std::cout << "\nVertex " << iter.first << " neighbors:";
      std::vector<int> neighbors;
      for (const auto& jter : dodec)
      {
        auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
        if (neighborSqDist == edgeLengthSq)
        {
          neighbors.push_back(jter.first);
          std::cout << "\n  Vertex " << jter.first << " at sq. distance " << neighborSqDist.print();
        }
      }
      adjacencyGraph[iter.first] = neighbors;
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::set<std::pair<int, int> > halfEdges;

    for (const auto& iter : adjacencyGraph)
    {
      for (const auto& index : iter.second) { halfEdges.insert({ iter.first, index }); }
    }

    for (const auto& halfEdge : halfEdges)
    {
      std::cout << "\nHalf-edge: " << halfEdge.first << " --> " << halfEdge.second;
    }
    std::cout << "\n\nNum. half-edges == " << halfEdges.size();
    std::cout << "\nNum. edges == " << halfEdges.size() / 2;

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::set<std::vector<int>, CompareFaces> faces;
    for (const auto& iter : adjacencyGraph)
    {
      std::vector<std::vector<int> > facesFrom;
      facesFrom.push_back({ iter.second[0], iter.first, iter.second[1] });
      facesFrom.push_back({ iter.second[1], iter.first, iter.second[2] });
      facesFrom.push_back({ iter.second[2], iter.first, iter.second[0] });
      for (const auto& faceFrom : facesFrom)
      {
        auto current = faceFrom;
        for (int verticesRemaining = 0; verticesRemaining < 2; ++verticesRemaining)
        {
          int nextOne = -1;
          for (const auto& subsequent : adjacencyGraph[current[current.size() - 1]])
          {
            bool doNotAdd = false;
            for (int ii = 0; ii < (int)current.size(); ++ii)
            {
              if (subsequent != current[ii]) { continue; }
              doNotAdd = true; break;
            }
            if (doNotAdd) { continue; }
            if (nextOne == -1) { nextOne = subsequent; continue; }
            if ((dodec[current[0]] - dodec[subsequent]).matrixSqNorm()
              < (dodec[current[0]] - dodec[nextOne]).matrixSqNorm())
            {
              nextOne = subsequent; continue;
            }
          }
          current.push_back(nextOne);
        }
        faces.insert(current);
      }
    }

    for (const auto& face : faces)
    {
      std::cout << "\nFace: (";
      int ii = -1;
      for (const auto& vert : face)
      {
        ++ii;
        if (ii > 0) { std::cout << ", "; }
        std::cout << vert;
      }
      std::cout << ")";
    }
    std::cout << "\n\nNum. faces == " << faces.size();

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::cout << "\n\nEuler characteristic ==\nNum. vertices - num. edges + num. faces == " <<
      dodec.size() - halfEdges.size() / 2 + faces.size() << "\n";

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::cout << "\nExporting dodecahedron to .obj format. Continue, Y or N?  ";
    std::cin >> prompt;
    if ((prompt.compare("N") == 0) || (prompt.compare("n") == 0)) { return true; }

    std::cout << "\nWriting..." << std::endl;
    bool wrote = exportDodecahedronObj("dodecahedron.obj");
    std::cout << (wrote ? "Export succeeded.\n" : "Export failed.\n");

    return true;
  }

  bool test_icosahedron()
  {
    std::string prompt;
    BiquadraticNumber edgeLength(Rational(1, 1));
    auto edgeLengthSq = edgeLength * edgeLength;

    std::cout << "\nICOSAHEDRON centered at (0, 0, 0) with all edge lengths == " << edgeLength.print() << ":";
    std::map<int, Matrix<BiquadraticNumber> > icosa;
    {
      int ii = -1;
      // Creates an icosahedron from a dual dodecahedron.
      auto icosaSet = getIcosahedron(edgeLength);  // Uses cached values from a call to getIcosahedronViaDual().
      //auto icosaSet = getIcosahedronViaDual(edgeLength);
      for (const auto& vertex : icosaSet)
      {
        ++ii;
        std::cout << "\nVertex " << ii << " = " << vertex.transpose().print(true);
        icosa[ii] = vertex;
      }
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    for (const auto& iter : icosa)
    {
      std::cout << "\nVertex " << iter.first << " sq. length = " << iter.second.matrixSqNorm().print();
    }

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::map<int, std::vector<int> > adjacencyGraph;

    for (const auto& iter : icosa)
    {
      std::cout << "\nVertex " << iter.first << " neighbors:";
      std::vector<int> neighbors;
      for (const auto& jter : icosa)
      {
        auto neighborSqDist = (iter.second - jter.second).matrixSqNorm();
        if (neighborSqDist == edgeLengthSq)
        {
          neighbors.push_back(jter.first);
          std::cout << "\n  Vertex " << jter.first << " at sq. distance " << neighborSqDist.print();
        }
      }
      adjacencyGraph[iter.first] = neighbors;
    }
    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::set<std::pair<int, int> > halfEdges;

    for (const auto& iter : adjacencyGraph)
    {
      for (const auto& index : iter.second) { halfEdges.insert({ iter.first, index }); }
    }

    for (const auto& halfEdge : halfEdges)
    {
      std::cout << "\nHalf-edge: " << halfEdge.first << " --> " << halfEdge.second;
    }
    std::cout << "\n\nNum. half-edges == " << halfEdges.size();
    std::cout << "\nNum. edges == " << halfEdges.size() / 2;

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::set<std::vector<int>, CompareFaces> faces;
    for (const auto& iter : adjacencyGraph)
    {
      int faceLim = 5;
      for (int ii = 0; ii < faceLim; ++ii)
      {
        int jj = (ii == faceLim - 1) ? 0 : (ii + 1);
        auto aa = iter.second[ii];
        auto bb = iter.second[jj];
        if ((halfEdges.find({ aa, bb }) == halfEdges.end()) && (halfEdges.find({ bb, aa }) == halfEdges.end()))
        {
          continue;
        }
        faces.insert({ iter.second[ii], iter.first, iter.second[jj] });
      }
    }

    for (const auto& face : faces)
    {
      std::cout << "\nFace: (";
      int ii = -1;
      for (const auto& vert : face)
      {
        ++ii;
        if (ii > 0) { std::cout << ", "; }
        std::cout << vert;
      }
      std::cout << ")";
    }
    std::cout << "\n\nNum. faces == " << faces.size();

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::cout << "\n\nEuler characteristic ==\nNum. vertices - num. edges + num. faces == " <<
      icosa.size() - halfEdges.size() / 2 + faces.size() << "\n";

    std::cout << "\n\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    std::cout << "\nExporting icosahedron to .obj format. Continue, Y or N?  ";
    std::cin >> prompt;
    if ((prompt.compare("N") == 0) || (prompt.compare("n") == 0)) { return true; }

    std::cout << "\nWriting..." << std::endl;
    bool wrote = exportIcosahedronObj("icosahedron.obj");
    std::cout << (wrote ? "Export succeeded.\n" : "Export failed.\n");

    return true;
  }

  bool test_icosahedral_symmetries()
  {
    std::string prompt;
    std::cout << "\n\nTest icosahedral symmetries... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    auto syms = getIcosahedralSymmetries();

    std::cout << "\nNumber of icosahedral symmetries without reflections == " << syms.size();

    std::cout << "\nMore... or 'T' to end current test?  ";
    std::cin >> prompt;
    if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

    return true;
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

      std::cout << "\nExporting cube to .obj format. Continue, Y or N?  ";
      std::cin >> prompt;
      if ((prompt.compare("N") == 0) || (prompt.compare("n") == 0)) { return true; }

      std::cout << "\nWriting..." << std::endl;
      bool wrote = exportCubeObj("cube.obj");
      std::cout << (wrote ? "Export succeeded.\n" : "Export failed.\n");

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
