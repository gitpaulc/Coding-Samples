
#include <iostream>

#include "function.h"

using namespace FunctionalCalculator;

bool test_evaluation()
{
  auto unit = ComplexQuadratic::sqrt(Rational(1, 1));
  auto oneTwelfth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 12)));
  auto oneSixth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 6)));
  auto oneFourth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 4)));
  auto oneThird = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 3)));
  auto half = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 2)));
  PiRational one(PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 1))));
  {
    FnPolynomial fn = FnPolynomial::cosATimesPiX(one, unit);
    std::cout << "\n\ncos(pi * x) = " << fn.print();
    FnPolynomial result;
    bool success = fn.tryEvaluateAtX(unit - unit, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(0) = " << result.print();
    success = fn.tryEvaluateAtX(half, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi / 2) = " << result.print();
    success = fn.tryEvaluateAtX(oneThird, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi / 3) = " << result.print();
    success = fn.tryEvaluateAtX(oneFourth, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi / 4) = " << result.print();
    success = fn.tryEvaluateAtX(oneSixth, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi / 6) = " << result.print();
    success = fn.tryEvaluateAtX(oneTwelfth, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi / 12) = " << result.print();
    success = fn.tryEvaluateAtX(unit, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(pi) = " << result.print();
    success = fn.tryEvaluateAtX(unit + half, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(3 * pi / 2) = " << result.print();
    success = fn.tryEvaluateAtX(unit + unit, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(2 * pi) = " << result.print();
    success = fn.tryEvaluateAtX(-oneSixth, result);
    if (!success) { return false; }
    std::cout << "\n\ncos(-pi / 6) = " << result.print();
  }

  std::string prompt = "";
  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  {
    FnPolynomial fn = FnPolynomial::sinATimesPiX(one, unit);
    std::cout << "\n\nsin(pi * x) = " << fn.print();
    FnPolynomial result;
    bool success = fn.tryEvaluateAtX(unit - unit, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(0) = " << result.print();
    success = fn.tryEvaluateAtX(half, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi / 2) = " << result.print();
    success = fn.tryEvaluateAtX(oneThird, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi / 3) = " << result.print();
    success = fn.tryEvaluateAtX(oneFourth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi / 4) = " << result.print();
    success = fn.tryEvaluateAtX(oneSixth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi / 6) = " << result.print();
    success = fn.tryEvaluateAtX(oneTwelfth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi / 12) = " << result.print();
    success = fn.tryEvaluateAtX(unit, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi) = " << result.print();
    success = fn.tryEvaluateAtX(unit + half, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(3 * pi / 2) = " << result.print();
    success = fn.tryEvaluateAtX(unit + unit, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(2 * pi) = " << result.print();
    success = fn.tryEvaluateAtX(-oneSixth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(-pi / 6) = " << result.print();
  }

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  {
    FnPolynomial fn = FnPolynomial::sinPi_AX_plus_BY_plus_CZ(one, oneThird, oneThird, oneThird);
    PiRational result;
    bool success = fn.tryEvaluateAtXYZ(unit, half, oneFourth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi/3 + pi/6 + pi/12) = cos(pi / 12) = " << result.print();
    success = fn.tryEvaluateAtXYZ(unit, -half, oneFourth, result);
    if (!success) { return false; }
    std::cout << "\n\nsin(pi/3 - pi/6 + pi/12) = sin(pi / 4) = " << result.print();
  }

  std::cout << "\n";
  return true;
}

bool test_composition()
{
  PiRational one(PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 1))));
  auto zero = one - one;
  {
    auto two = one + one;
    FnPolynomial fn = FnPolynomial::multinomial(one, two, two + one, two + two, one, 2);
    std::cout << "\n\n(1 + 2x + 3y + 4z)^2 = " << fn.print();
    auto fnOther = FnPolynomial::multinomial(one, one, zero, zero, one, 2);
    std::cout << "\n\n(1 + x)^2 = " << fnOther.print();
    Matrix<ComplexQuadratic> transform;
    transform.addRow({ ComplexQuadratic::sqrt(Rational(4, 1)), ComplexQuadratic::sqrt(Rational(9, 1)), ComplexQuadratic::sqrt(Rational(16, 1)) });
    transform.addRow({ ComplexQuadratic::sqrt(Rational(0, 1)), ComplexQuadratic::sqrt(Rational(0, 1)), ComplexQuadratic::sqrt(Rational(0, 1)) });
    transform.addRow({ ComplexQuadratic::sqrt(Rational(0, 1)), ComplexQuadratic::sqrt(Rational(0, 1)), ComplexQuadratic::sqrt(Rational(0, 1)) });
    fnOther = fnOther.composeWith(transform);
    std::cout << "\n\n(1 + 2x + 3y + 4z)^2 = " << fnOther.print();
    std::cout << "\n0 = " << (fn - fnOther).print();
  }
  mp frequency(1);
  auto sine = FnPolynomial::sinATimesPiX(one, ComplexQuadratic(Rational(frequency, 1)));
  auto cosine = FnPolynomial::cosATimesPiX(one, ComplexQuadratic(Rational(frequency, 1)));
  PiRational eigen;
  bool isEigen = sine.isLaplaceEigenfunction(eigen);
  if (isEigen) { std::cout << "\n\n" << sine.print() << " is a Laplace eigenfunction with eigenvalue " << eigen.print(); }
  isEigen = cosine.isLaplaceEigenfunction(eigen);
  if (isEigen) { std::cout << "\n\n" << cosine.print() << " is a Laplace eigenfunction with eigenvalue " << eigen.print(); }

  std::string prompt = "";
  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  std::set<Matrix<ComplexQuadratic> > cubeVertices;
  {
    ComplexQuadratic unit(Rational(1, 1));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ -unit, -unit, -unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ -unit, -unit, unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ -unit, unit, -unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ -unit, unit, unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ unit, -unit, -unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ unit, -unit, unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ unit, unit, -unit }));
    cubeVertices.insert(Matrix<ComplexQuadratic>({ unit, unit, unit }));
  }

  FnPolynomial cubeSine, cubeCosine;
  for (const auto& vertex : cubeVertices)
  {
    ComplexQuadratic zero(Rational(0, 1));
    auto transform = vertex;
    transform.addRow({ zero, zero, zero });
    transform.addRow({ zero, zero, zero });
    cubeSine = cubeSine + sine.composeWith(transform);
    cubeCosine = cubeCosine + cosine.composeWith(transform);
  }
  isEigen = cubeSine.isLaplaceEigenfunction(eigen);
  if (isEigen) { std::cout << "\n\n" << cubeSine.print() << " is a Laplace eigenfunction with eigenvalue " << eigen.print(); }
  isEigen = cubeCosine.isLaplaceEigenfunction(eigen);
  if (isEigen) { std::cout << "\n\nF(x, y, z) = " << cubeCosine.print() << " is a Laplace eigenfunction with eigenvalue " << eigen.print(); }

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  {
    ComplexQuadratic half(Rational(1, 2));
    FnPolynomial result;
    bool success = cubeCosine.tryEvaluateAtX(-half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(-1/2, y, z) = " << result.print();
    success = cubeCosine.tryEvaluateAtX(half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(1/2, y, z) = " << result.print();
    success = cubeCosine.tryEvaluateAtY(-half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(x, -1/2, z) = " << result.print();
    success = cubeCosine.tryEvaluateAtY(half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(x, 1/2, z) = " << result.print();
    success = cubeCosine.tryEvaluateAtZ(-half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(x, y, -1/2) = " << result.print();
    success = cubeCosine.tryEvaluateAtZ(half, result);
    if (!success) { return false; }
    std::cout << "\n\nF(x, y, 1/2) = " << result.print();
  }

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  {
    ComplexQuadratic unit = ComplexQuadratic(QuadraticNumber(1));
    auto zippo = unit - unit;
    Matrix<ComplexQuadratic> ninetyDegreeX;
    ninetyDegreeX.addRow({ unit, zippo, zippo });
    ninetyDegreeX.addRow({ zippo, zippo, unit });
    ninetyDegreeX.addRow({ zippo, -unit, zippo });
    std::cout << "\n\nA = Fix x-axis, Rotation by angle pi / 2:\n" << ninetyDegreeX.print(true);
    Matrix<ComplexQuadratic> ninetyDegreeY;
    ninetyDegreeY.addRow({ zippo, zippo, unit });
    ninetyDegreeY.addRow({ zippo, unit, zippo });
    ninetyDegreeY.addRow({ -unit, zippo, zippo });
    std::cout << "\n\nB = Fix y-axis, Rotation by angle pi / 2:\n" << ninetyDegreeY.print(true);
    Matrix<ComplexQuadratic> ninetyDegreeZ;
    ninetyDegreeZ.addRow({ zippo, unit, zippo });
    ninetyDegreeZ.addRow({ -unit, zippo, zippo });
    ninetyDegreeZ.addRow({ zippo, zippo, unit });
    std::cout << "\n\nA = Fix z-axis, Rotation by angle pi / 2:\n" << ninetyDegreeZ.print(true);
    std::cout << "\n\nF(x, y, z) = " << cubeCosine.print();
    auto cubeCosine_ = cubeCosine.composeWith(ninetyDegreeX);
    std::cout << "\n\nF(A(x, y, z)) - F(x, y, z) = " << (cubeCosine_ - cubeCosine).print(true);
    cubeCosine_ = cubeCosine.composeWith(ninetyDegreeY);
    std::cout << "\n\nF(B(x, y, z)) - F(x, y, z) = " << (cubeCosine_ - cubeCosine).print(true);
    cubeCosine_ = cubeCosine.composeWith(ninetyDegreeZ);
    std::cout << "\n\nF(C(x, y, z)) - F(x, y, z) = " << (cubeCosine_ - cubeCosine).print(true);
  }

  return true;
}

bool test_matrix()
{
  {
    Matrix<QuadraticNumber> rot2PiOver3;
    rot2PiOver3.addRow({ Rational(-1, 2), QuadraticNumber::sqrt(3) * Rational(-1, 2) });
    rot2PiOver3.addRow({ QuadraticNumber::sqrt(3) * Rational(1, 2) , Rational(-1, 2) });
    QuadraticNumber det; bool linInd = true;
    auto rref = rot2PiOver3.rref(det, linInd);
    std::cout << "\nRotation by angle 2 * pi / 3:\n" << rot2PiOver3.print(true);
    std::cout << "\nIts determinant = " << det.print();

    Matrix<QuadraticNumber> rot2 = rot2PiOver3 * rot2PiOver3;
    std::cout << "\n\nRotation by angle 4 * pi / 3:\n" << rot2.print(true);
    Matrix<QuadraticNumber> id = rot2PiOver3 * rot2PiOver3 * rot2PiOver3;
    std::cout << "\n\nRotation by angle 6 * pi / 3:\n" << id.print(true);
    std::cout << "\n\nIdentity matrix = \n" << rref.print(true);
  }

  std::string prompt = "";
  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  Matrix<QuadraticNumber> rot2PiOver3;
  rot2PiOver3.addRow({ Rational(1), Rational(0), Rational(0) });
  rot2PiOver3.addRow({ Rational(0), Rational(-1, 2), QuadraticNumber::sqrt(3) * Rational(-1, 2) });
  rot2PiOver3.addRow({ Rational(0), QuadraticNumber::sqrt(3) * Rational(1, 2) , Rational(-1, 2) });

  // Elementary row operations test. These should ultimately leave the matrix unchanged:
  rot2PiOver3.swapRows(0, 1); rot2PiOver3.swapRows(1, 0);
  rot2PiOver3.scaleRow(1, Rational(2, 1)); rot2PiOver3.scaleRow(1, Rational(1, 2));
  rot2PiOver3.addScaledRowJ_toI(0, 1, Rational(2, 1));
  rot2PiOver3.addScaledRowJ_toI(0, 1, Rational(-2, 1));

  std::cout << "\n\nRotation (R) by angle 2 * pi / 3:\n" << rot2PiOver3.print(true);

  Matrix<QuadraticNumber> rot2 = rot2PiOver3 * rot2PiOver3;
  std::cout << "\n\nRotation (R^2) by angle 4 * pi / 3:\n" << rot2.print(true);
  std::cout << "\n\nR^2 == R^T:\n" << rot2PiOver3.transpose().print(true);
  Matrix<QuadraticNumber> id = rot2PiOver3 * rot2PiOver3 * rot2PiOver3;
  std::cout << "\n\nRotation (R^3) by angle 6 * pi / 3:\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  Matrix<QuadraticNumber> otherRot2PiOver3;
  otherRot2PiOver3.addRow({ Rational(-1, 3), QuadraticNumber::sqrt(Rational(2, 3)) * Rational(-1), QuadraticNumber::sqrt(Rational(2)) * Rational(-1, 3) });
  otherRot2PiOver3.addRow({ QuadraticNumber::sqrt(Rational(2, 3)), Rational(-1, 2) , QuadraticNumber::sqrt(Rational(1, 3)) * Rational(1, 2) });
  otherRot2PiOver3.addRow({ QuadraticNumber::sqrt(Rational(2)) * Rational(-1, 3), QuadraticNumber::sqrt(Rational(1, 3)) * Rational(-1, 2), Rational(5, 6)});
  std::cout << "\n\nRotation (P) by angle 2 * pi / 3:\n" << otherRot2PiOver3.print(true);

  Matrix<QuadraticNumber> other2 = otherRot2PiOver3 * otherRot2PiOver3;
  std::cout << "\n\nRotation (P^2) by angle 4 * pi / 3:\n" << other2.print(true);
  {
    bool success = false;
    std::cout << "\n\nP^2 == P^{-1}:\n" << otherRot2PiOver3.inverse(success).print(true);
  }
  id = otherRot2PiOver3 * otherRot2PiOver3 * otherRot2PiOver3;
  std::cout << "\n\nRotation (P^3) by angle 6 * pi / 3:\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  auto rTimesP = rot2PiOver3 * otherRot2PiOver3;
  std::cout << "\n\nRotation (R * P) by angle 2 * pi / 3:\n" << rTimesP.print(true);
  Matrix<QuadraticNumber> rTimesP_2 = rTimesP * rTimesP;
  std::cout << "\n\nRotation ((R * P)^2) by angle 4 * pi / 3:\n" << rTimesP_2.print(true);
  id = rTimesP * rTimesP * rTimesP;
  std::cout << "\n\nRotation ((R * P)^3) by angle 6 * pi / 3:\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  auto pTimesR = otherRot2PiOver3 * rot2PiOver3;
  std::cout << "\n\nRotation (P * R) by angle 2 * pi / 3:\n" << pTimesR.print(true);
  Matrix<QuadraticNumber> pTimesR_2 = pTimesR * pTimesR;
  std::cout << "\n\nRotation ((P * R)^2) by angle 4 * pi / 3:\n" << pTimesR_2.print(true);
  id = pTimesR * pTimesR * pTimesR;
  std::cout << "\n\nRotation ((P * R)^3) by angle 6 * pi / 3:\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  auto pTimesRTimesP = otherRot2PiOver3 * rot2PiOver3 * otherRot2PiOver3;
  std::cout << "\n\n(P * R * P) =\n" << pTimesRTimesP.print(true);
  pTimesRTimesP = rot2PiOver3 * otherRot2PiOver3 * rot2PiOver3;
  std::cout << "\n\n(R * P * R) =\n" << pTimesRTimesP.print(true);
  id = pTimesRTimesP * pTimesRTimesP;
  std::cout << "\n\n(P * R * P) * (P * R * P) =\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  auto pTimesR2 = otherRot2PiOver3 * rot2;
  std::cout << "\n\n(P * R^2) =\n" << pTimesR2.print(true);
  pTimesR2 = rot2PiOver3 * otherRot2PiOver3 * otherRot2PiOver3;
  std::cout << "\n\n(R * P^2) =\n" << pTimesR2.print(true);
  id = pTimesR2 * pTimesR2;
  std::cout << "\n\n(P * R^2) * (P * R^2) =\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  auto r2TimesP = rot2 * otherRot2PiOver3;
  std::cout << "\n\n(R^2 * P) =\n" << r2TimesP.print(true);
  r2TimesP = otherRot2PiOver3 * otherRot2PiOver3 * rot2PiOver3;
  std::cout << "\n\n(P^2 * R) =\n" << r2TimesP.print(true);
  id = r2TimesP * r2TimesP;
  std::cout << "\n\n(R^2 * P) * (R^2 * P) =\n" << id.print(true);

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }

  std::set<Matrix<QuadraticNumber> > tetrahedralSymmetries;
  tetrahedralSymmetries.insert(id);
  tetrahedralSymmetries.insert(rot2PiOver3);
  tetrahedralSymmetries.insert(rot2);
  tetrahedralSymmetries.insert(otherRot2PiOver3);
  tetrahedralSymmetries.insert(other2);
  tetrahedralSymmetries.insert(rTimesP);
  tetrahedralSymmetries.insert(rTimesP_2);
  tetrahedralSymmetries.insert(pTimesR);
  tetrahedralSymmetries.insert(pTimesR_2);
  tetrahedralSymmetries.insert(pTimesRTimesP);
  tetrahedralSymmetries.insert(pTimesR2);
  tetrahedralSymmetries.insert(r2TimesP);
  std::cout << "\nNumber of tetrahedral (orientation-preserving) symmetries: " << tetrahedralSymmetries.size();
  bool passedClosedness = true;
  for (const auto& sym0 : tetrahedralSymmetries)
  {
    std::set<Matrix<QuadraticNumber> > newSymmetries;
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
    std::set<Matrix<QuadraticNumber> > newSymmetries;
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
    Matrix<QuadraticNumber> vec0;
    vec0.addRow({ QuadraticNumber::sqrt(6) * Rational(1, 4), Rational(0), Rational(0) });
    vec0 = vec0.transpose();
    std::set<Matrix<QuadraticNumber> > tetrahedron;
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

bool test_mp()
{
  mp zero = 0;
  std::cout << "\n0 = " << zero;
  mp one = 1;
  mp two = one + one;
  std::cout << "\n1 = " << one << "\n2 = " << two;
  mp twoToThe16 = two.pow(16);
  std::cout << "\n65,536 = " << twoToThe16;
  mp thousand = 1000;
  std::cout << "\n1000 = " << thousand;
  auto million = mp(500000) + mp(500000);
  std::cout << "\n1 million = " << million;
  million = thousand * thousand;
  std::cout << "\n1 million = " << million;
  std::cout << "\n1,000,001 = " << (million + one);
  std::cout << "\n1 trillion = " << million * million;
  auto squareOf_65536 = (twoToThe16 * twoToThe16);
  std::cout << "\n65,536^2 = " << squareOf_65536;
  auto squareOf_8192 = mp(8192) * mp(8192);
  std::cout << "\n8192^2 = " << (squareOf_8192);
  std::cout << "\n7th digit of 8192^2 (from the right) = " << squareOf_8192.getDigit(6);
  auto replaced = squareOf_8192;
  replaced.setDigit(6, 5);
  std::cout << "\nReplace 7th digit to 5 in 8192^2 = " << replaced;
  replaced = squareOf_8192;
  replaced.setDigit(7, 5);
  std::cout << "\nReplace 8th digit to 5 in 8192^2 = " << replaced;
  std::cout << "\nNumber of digits in " << squareOf_65536 << " = " << squareOf_65536.numDigits();
  std::cout << "\n10^6 - 500,000 = " << million - mp(500000);
  std::cout << "\n500,000 - 10^6 = " << mp(500000) - million;
  {
    mp divisor(2);
    for (int ii = 31; ii >= 0; --ii)
    {
      std::cout << "\n2^" << ii << " = " << squareOf_65536 / divisor;
      std::cout << " with a remainder of " << squareOf_65536 % divisor;
      divisor = divisor * mp(2);
    }
  }
  std::cout << "\n" << squareOf_8192 << " / " << mp(11) << " = " << (squareOf_8192 / mp(11)) << " with a remainder of " << (squareOf_8192 % mp(11));
  return true;
}

bool test_rational()
{
  Rational zero;
  std::cout << "\nZero = " << zero.print();
  Rational one;
  one = one + Rational(1, 2);
  one = one + Rational(1, 3);
  one = one + Rational(1, 6);
  std::cout << "\nOne = " << one.print();
  std::cout << "\nPrime factorization of one = " << one.printFactors();
  std::cout << "\nPrime factorization of -1 = " << (-one).printFactors();
  Rational half = Rational(-1, 4) * Rational(4, -2);
  std::cout << "\nOne half = " << half.print();
  Rational thePower = half;
  for (int i = 0; i < 6; ++i)
  {
    auto newPower = thePower * thePower;
    std::cout << "\n" << thePower.print() << "^2 = " << newPower.print();
    thePower = newPower;
  }
  Rational twelve = Rational(36, 3);
  std::cout << "\nPrime factorization of twelve = " << twelve.printFactors();
  Rational minusTwelve = Rational(24, -2);
  std::cout << "\nPrime factorization of negative twelve = " << minusTwelve.printFactors();
  Rational oneOver2048 = Rational(2, 4096);
  std::cout << "\nPrime factorization of 1 / 2048 = " << oneOver2048.printFactors();
  Rational hundred = Rational(1000, 10);
  std::cout << "\nPrime factorization of 100 = " << hundred.printFactors();
  Rational myNum = Rational(-24, 138);
  std::cout << "\nPrime factorization of -24 / 138 = " << myNum.printFactors();
  return true;
}

bool test_quadratic()
{
  auto zero = QuadraticNumber();
  std::cout << "\nZero = " << zero.print();
  zero = QuadraticNumber::sqrt(9) - Rational(3);
  std::cout << "\nZero = " << zero.print();
  auto twoThirds = QuadraticNumber(Rational(2, 3));
  std::cout << "\nTwo-thirds = " << twoThirds.print();
  auto one = QuadraticNumber::sqrt(1);
  std::cout << "\nSquare root of 1 = " << one.print();
  auto sqrt2 = QuadraticNumber::sqrt(2);
  std::cout << "\nSquare root of 2 = " << sqrt2.print();
  auto sqrt36 = QuadraticNumber::sqrt(36);
  std::cout << "\nSquare root of 36 = " << sqrt36.print();
  auto sqrt12 = QuadraticNumber::sqrt(12);
  std::cout << "\nSquare root of 12 = " << sqrt12.print();
  std::cout << "\nTwelve is " << (sqrt12 * sqrt12).print();
  Rational rationalOut;
  bool twoThirdsIsRational = twoThirds.getRational(rationalOut);
  if (!twoThirdsIsRational) { return false; }
  std::cout << "\nSquare root of 2/3 = " << QuadraticNumber::sqrt(rationalOut).print();
  auto goldenRatio = QuadraticNumber::sqrt(Rational(5, 4)) + Rational(1, 2);
  std::cout << "\nThe golden ratio is " << goldenRatio.print();
  auto oneOverGolden = QuadraticNumber::sqrt(Rational(5, 4)) - Rational(1, 2);
  std::cout << "\nOne = " << (goldenRatio * oneOverGolden).print();
  oneOverGolden = QuadraticNumber(1) / goldenRatio;
  std::cout << "\nThe reciprocal golden ratio is " << oneOverGolden.print();
  auto sumOfSquareRoots = QuadraticNumber::sqrt(2) + QuadraticNumber::sqrt(3) + Rational(1);
  auto reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\nThe reciprocal of " << sumOfSquareRoots.print() << " is " << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();
  sumOfSquareRoots = QuadraticNumber::sqrt(5) - QuadraticNumber::sqrt(3) + Rational(1);
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();
  sumOfSquareRoots = QuadraticNumber::sqrt(7) + QuadraticNumber::sqrt(3) + Rational(1);
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();
  sumOfSquareRoots = QuadraticNumber::sqrt(7) + QuadraticNumber::sqrt(5) + QuadraticNumber::sqrt(3) + Rational(1);
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();

  std::string prompt;
  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 5; ++ii)
  {
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii);
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print() << std::endl;
  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 6; ++ii)
  {
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii);
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print() << std::endl;
  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 6; ++ii)
  {
    QuadraticNumber coeff(-1);
    if ((ii % 2) == 0) { coeff = coeff * coeff; }
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii) * coeff;
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  std::cout << "\nOne = " << (reciprocal * sumOfSquareRoots).print();

  std::cout << "\n\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 5; ++ii)
  {
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii);
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  auto recipIntegral = reciprocal.factorAsIntegral();
  std::cout << "\nThis equals " << recipIntegral.first.print(true) << " / " << recipIntegral.second;
  std::cout << "\nOne = " << (sumOfSquareRoots * recipIntegral.first * Rational(1, recipIntegral.second)).print() << std::endl;
  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 6; ++ii)
  {
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii);
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  recipIntegral = reciprocal.factorAsIntegral();
  std::cout << "\nThis equals " << recipIntegral.first.print(true) << " / " << recipIntegral.second;
  std::cout << "\nOne = " << (sumOfSquareRoots * recipIntegral.first * Rational(1, recipIntegral.second)).print() << std::endl;

  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 6; ++ii)
  {
    QuadraticNumber coeff(-1);
    if ((ii % 2) == 0) { coeff = coeff * coeff; }
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii) * coeff;
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  recipIntegral = reciprocal.factorAsIntegral();
  std::cout << "\nThis equals " << recipIntegral.first.print(true) << " / " << recipIntegral.second;
  std::cout << "\nOne = " << (sumOfSquareRoots * recipIntegral.first * Rational(1, recipIntegral.second)).print() << std::endl;

  sumOfSquareRoots = QuadraticNumber();
  for (int ii = 0; ii < 6; ++ii)
  {
    QuadraticNumber coeff(-1);
    if ((ii % 2) == 1) { coeff = coeff * coeff; }
    sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii) * coeff;
  }
  reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
  std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
  recipIntegral = reciprocal.factorAsIntegral();
  std::cout << "\nThis equals " << recipIntegral.first.print(true) << " / " << recipIntegral.second;
  std::cout << "\nOne = " << (sumOfSquareRoots * recipIntegral.first * Rational(1, recipIntegral.second)).print() << std::endl;

  std::cout << "\nMore... or 'T' to end current test?  ";
  std::cin >> prompt;
  if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
  for (int lim_ = 7; lim_ < 11; ++lim_)
  {
    if (lim_ == 9)
    {
      std::cout << "\n\nThe next part of this test is slow.";
      std::cout << "\nMore... or 'T' to end current test?  ";
      std::cin >> prompt;
      if ((prompt.compare("T") == 0) || (prompt.compare("t") == 0)) { return true; }
    }
    sumOfSquareRoots = QuadraticNumber();
    for (int ii = 0; ii < lim_; ++ii)
    {
      sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber::sqrt(ii);
    }
    reciprocal = QuadraticNumber(1) / sumOfSquareRoots;
    std::cout << "\n\nThe reciprocal of " << sumOfSquareRoots.print() << " is:\n" << reciprocal.print();
    recipIntegral = reciprocal.factorAsIntegral();
    std::cout << "\nThis equals " << recipIntegral.first.print(true) << " / " << recipIntegral.second;
    std::cout << "\nOne = " << (sumOfSquareRoots * recipIntegral.first * Rational(1, recipIntegral.second)).print() << std::endl;
  }

  return true;
}

bool test_complex()
{
  auto ii = ComplexQuadratic::sqrt(-1);
  std::cout << "\nSquare root of -1 = " << ii.print();
  auto sqrtI = ComplexQuadratic::sqrtOfITimes(1);
  std::cout << "\nSquare root of i = " << sqrtI.print();
  std::cout << "\ni = " << (sqrtI * sqrtI).print();
  auto sqrtMinus12 = ComplexQuadratic::sqrt(-12);
  std::cout << "\nSquare root of -12 = " << sqrtMinus12.print();
  auto sqrtMinus36 = ComplexQuadratic::sqrt(-36);
  std::cout << "\nSquare root of -36 = " << sqrtMinus36.print();
  std::cout << "\nThe reciprocal of i is " << (ComplexQuadratic::sqrt(-1).pow(-1)).print();
  auto rootThreeNum = (ComplexQuadratic::sqrt(-3) + QuadraticNumber(1)) / QuadraticNumber(2);
  std::cout << "\nThe following equation holds:\n" << rootThreeNum.print(true) << " * " << rootThreeNum.conjugate().print(true);
  std::cout << " = " << (rootThreeNum * rootThreeNum.conjugate()).print() << std::endl;
  return true;
}

bool test_pi()
{
  auto piPoly = PiPolynomial();
  std::cout << "\n0 * pi^0 = " << piPoly.print();
  piPoly = PiPolynomial(ComplexQuadratic::sqrt(-1), 2);
  std::cout << "\ni * pi^2 = " << piPoly.print();
  piPoly = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(1))});
  auto piPoly1 = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(-1))});
  auto piPoly2 = PiPolynomial({ComplexQuadratic(Rational(1)), ComplexQuadratic(Rational(0)), ComplexQuadratic(Rational(1))});
  std::cout << "\n1 + pi = " << piPoly.print();
  std::cout << "\n1 - pi = " << piPoly1.print();
  std::cout << "\n1 + pi^2 = " << piPoly2.print();
  auto oneMinusPiSq = piPoly * piPoly1;
  std::cout << "\n1 - pi^2 = " << oneMinusPiSq.print();
  auto product = piPoly * piPoly1 * piPoly2;
  std::cout << "\n1 - pi^4 = " << product.print();
  auto one = product + PiPolynomial(ComplexQuadratic(1), 4);
  std::cout << "\n1 = " << one.print();
  PiPolynomial remainder;
  PiPolynomial divisor = one + PiPolynomial(2, 3);
  auto quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  divisor = piPoly1;
  quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  product = PiPolynomial(4) - PiPolynomial(9, 4);
  divisor = PiPolynomial(ComplexQuadratic::sqrt(2)) - PiPolynomial(ComplexQuadratic::sqrt(3), 1);
  quotient = product.division(divisor, remainder);
  std::cout << "\n\nDivision: [" << product.print() << "] / [" << divisor.print() << "] = " << quotient.print();
  std::cout << "\nThe remainder of [" << product.print() << "] / [" << divisor.print() << "] is " << remainder.print();
  product = piPoly * piPoly2;
  std::cout << "\n\nThe gcd of " << product.print() << " and " << oneMinusPiSq.print();
  std::cout << " is " << PiPolynomial::gcd(product, oneMinusPiSq).print();
  std::cout << "\nThe gcd of " << oneMinusPiSq.print() << " and " << product.print();
  std::cout << " is " << PiPolynomial::gcd(oneMinusPiSq, product).print();
  return true;
}

bool test_fn_poly()
{
  {
    FnPolynomial sineOfPiX = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    std::cout << "\n\nsin(pi * x) = " << sineOfPiX.print();
    FnPolynomial piCosPiX = sineOfPiX.partial_x();
    std::cout << "\npi * cos(pi * x) = " << piCosPiX.print();

    // The calculator deduces sin^2 + cos^2 = 1:
    auto one = sineOfPiX * sineOfPiX + (piCosPiX * piCosPiX) * (PiRational(ComplexQuadratic(1), PiPolynomial(1, 2)));
    std::cout << "\n\nsin^2(pi * x) + cos^2(pi * x) = " << one.print();
  }
  {
    std::cout << "\n\nTrig identities:";
    FnPolynomial sinPi2X = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 2);
    FnPolynomial cosPi2X = FnPolynomial::cosATimesPiX(PiPolynomial(ComplexQuadratic(1)), 2);
    FnPolynomial sinPiX = FnPolynomial::sinATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    FnPolynomial cosPiX = FnPolynomial::cosATimesPiX(PiPolynomial(ComplexQuadratic(1)), 1);
    auto identity1 = sinPi2X * (PiPolynomial(1) * Rational(1, 2)) - sinPiX * cosPiX;
    auto identity2 = cosPi2X - cosPiX * cosPiX + sinPiX * sinPiX;
    std::cout << "\nsin(2 * pi * x) / 2 - sin(pi * x) * cos(pi * x) = " << identity1.print();
    std::cout << "\ncos(2 * pi * x) - cos^2(pi * x) + sin^2(pi * x) = " << identity2.print();
  }
  {
    auto harmonic = FnPolynomial::eToTheATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiY(PiPolynomial(2), 3);
    std::cout << "\n\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    auto notHarmonic = FnPolynomial::eToTheATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiX(PiPolynomial(2), 3);
    std::cout << "\nThe function " << notHarmonic.print() << " is " << (notHarmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  {
    auto efunc = FnPolynomial::sinATimesPiX(PiPolynomial(2), 3) * FnPolynomial::sinATimesPiY(PiPolynomial(2), 3);
    PiRational lambda;
    bool isEigen = efunc.isLaplaceEigenfunction(lambda);
    if (isEigen)
    {
      std::cout << "\n\nThe function " << efunc.print() << " is a Laplace eigenfunction with eigenvalue " << lambda.print() << ".";
    }
    auto notEfunc = efunc + FnPolynomial::sinATimesPiX(PiPolynomial(2), 3);
    PiRational shouldNotChange;
    isEigen = notEfunc.isLaplaceEigenfunction(shouldNotChange);
    if (!isEigen) { std::cout << "\n\nThe function " << notEfunc.print() << " is not a Laplace eigenfunction."; }
  }
  std::cout << "\n";
  return true;
}

bool test_function()
{
  auto xx = FnPolynomial::xToPower(PiPolynomial(1), 1);
  auto yy = FnPolynomial::yToPower(PiPolynomial(1), 1);
  std::cout << "\n";
  {
    auto tan = Function::tanATimesPiX(PiPolynomial(1), 1);
    std::cout << "\ntan(pi * x) = " << tan.print();
    auto unit = ComplexQuadratic::sqrt(Rational(1, 1));
    auto zippo = unit - unit;
    auto oneTwelfth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 12)));
    auto oneSixth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 6)));
    auto oneFourth = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 4)));
    auto oneThird = unit * ComplexQuadratic(QuadraticNumber(Rational(1, 3)));
    PiRational one(PiPolynomial(ComplexQuadratic::sqrt(Rational(1, 1))));
    PiRational expression;
    tan.tryEvaluateAtXYZ(unit, zippo, zippo, expression);
    std::cout << "\ntan(pi) = " << expression.print();
    tan.tryEvaluateAtXYZ(oneThird, zippo, zippo, expression);
    std::cout << "\ntan(pi / 3) = " << expression.print();
    tan.tryEvaluateAtXYZ(oneFourth, zippo, zippo, expression);
    std::cout << "\ntan(pi / 4) = " << expression.print();
    tan.tryEvaluateAtXYZ(oneSixth, zippo, zippo, expression);
    std::cout << "\ntan(pi / 6) = " << expression.print();
    tan.tryEvaluateAtXYZ(oneTwelfth, zippo, zippo, expression);
    std::cout << "\ntan(pi / 12) = " << expression.print();
    auto sec = Function(FnPolynomial(PiPolynomial(1)), FnPolynomial::cosATimesPiX(PiPolynomial(1), 1));
    auto sec2_times_pi = sec * sec * Function::constant(PiPolynomial(1, 1));
    std::cout << "\npi * sec^2(pi * x) - (d/dx)tan(pi * x) = " << (sec2_times_pi - tan.partial_x()).print();
  }
  {
    auto harmonic = Function(xx * xx - yy * yy);
    std::cout << "\n\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    harmonic = Function(xx, xx * xx + yy * yy);
    std::cout << "\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
    auto notHarmonic = Function(xx, xx * xx - yy * yy);
    std::cout << "\nThe function " << notHarmonic.print() << " is " << (notHarmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  {
    auto numerator = FnPolynomial::sinATimesPiX(PiPolynomial(1), 1) * FnPolynomial::cosATimesPiX(PiPolynomial(1), 1);
    auto coshTerm = FnPolynomial::coshATimesPiY(PiPolynomial(1), 1) * FnPolynomial::coshATimesPiY(PiPolynomial(1), 1);
    auto sinTerm = FnPolynomial::sinATimesPiX(PiPolynomial(1), 1) * FnPolynomial::sinATimesPiX(PiPolynomial(1), 1);
    auto denominator = coshTerm - sinTerm;
    auto harmonic = Function(numerator, denominator);
    std::cout << "\nThe function " << harmonic.print() << " is " << (harmonic.isHarmonic() ? "" : "not ") << "harmonic.";
  }
  return true;
}

int main()
{
  std::string prompt;
  std::cout << "\n\nTesting function evaluation:\n";
  test_evaluation();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTesting function composition:\n";
  test_composition();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTesting matrices:\n";
  test_matrix();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTesting functions:\n";
  test_function();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest function polynomials:\n";
  test_fn_poly();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\nTest rational:\n";
  test_rational();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest quadratic:\n";
  test_quadratic();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest complex:\n";
  test_complex();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTest pi polynomials:\n";
  test_pi();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
  std::cout << "\n\nTesting multiprecision integers:\n";
  test_mp();
  std::cout << "\nContinue, or 'Q' to exit? ";
  std::cin >> prompt;
  if ((prompt.compare("Q") == 0) || (prompt.compare("q") == 0)) { return 0; }
}
