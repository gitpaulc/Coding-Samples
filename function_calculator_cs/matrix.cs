/*  Copyright Paul Cernea, April 2026.
All Rights Reserved.*/

namespace function_calculator_cs
{

public interface IFieldElement<TSelf>
    where TSelf : IFieldElement<TSelf>
{
  TSelf Add(TSelf rhs);
  TSelf Sub(TSelf rhs);
  TSelf Mul(TSelf rhs);
  TSelf Div(TSelf rhs);
  TSelf Neg();
  bool IsZero();
  bool FieldEquals(TSelf other);
  int FieldCompareTo(TSelf other);
  string Print(bool useParentheses = false);

  static abstract TSelf FieldZero();
  static abstract TSelf FieldOne();
}

public class Matrix<T> : IComparable<Matrix<T>>
    where T : IFieldElement<T>
{
  private List<List<T>> rows = new List<List<T>>();

  private static int getLeadingOneIndex(List<T> row)
  {
    for (int ii = 0; ii < row.Count; ++ii)
    {
      if (!row[ii].IsZero()) { return ii; }
    }
    return row.Count;
  }

  public string Print(bool useParentheses = false)
  {
    if (rows.Count == 0) { return useParentheses ? "(0)" : "0"; }
    var sb = new System.Text.StringBuilder();
    int numR = rows.Count, numC = rows[0].Count;
    var buf = new string[numR, numC];
    var longest = new int[numC];
    for (int ii = 0; ii < numR; ++ii)
    {
      for (int jj = 0; jj < numC; ++jj)
      {
        var s = rows[ii][jj].Print(false);
        buf[ii, jj] = s;
        if (s.Length > longest[jj]) { longest[jj] = s.Length; }
      }
    }
    for (int ii = 0; ii < numR; ++ii)
    {
      sb.Append('\n');
      if (useParentheses) { sb.Append(numR == 1 ? "(" : "|| "); }
      for (int jj = 0; jj < numC; ++jj)
      {
        if (jj > 0) { sb.Append(numR == 1 ? ", " : " | "); }
        var cur = buf[ii, jj];
        sb.Append(cur);
        for (int k = cur.Length; k < longest[jj]; ++k) { sb.Append(' '); }
      }
      if (useParentheses) { sb.Append(numR == 1 ? ")" : " ||"); }
    }
    return sb.ToString();
  }

  public int NumRows() => rows.Count;
  public int NumCols() => rows.Count == 0 ? 0 : rows[0].Count;

  public static Matrix<T> ZeroMatrix(int rowDim, int colDim)
  {
    var m = new Matrix<T>();
    for (int ii = 0; ii < rowDim; ++ii)
    {
      var row = new List<T>(colDim);
      for (int jj = 0; jj < colDim; ++jj) { row.Add(T.FieldZero()); }
      m.AddRow(row);
    }
    return m;
  }

  public static Matrix<T> ZeroMatrix(int dim) => ZeroMatrix(dim, dim);

  public static Matrix<T> Identity(int dim)
  {
    var m = new Matrix<T>();
    for (int ii = 0; ii < dim; ++ii)
    {
      var row = new List<T>(dim);
      for (int jj = 0; jj < dim; ++jj)
      {
        row.Add(ii == jj ? T.FieldOne() : T.FieldZero());
      }
      m.AddRow(row);
    }
    return m;
  }

  public T At(int i, int j)
  {
    if (i < 0 || j < 0 || i >= NumRows() || j >= NumCols())
      throw new System.Exception("Matrix index out of bounds.");
    return rows[i][j];
  }

  public void SetAt(int i, int j, T val)
  {
    if (i < 0 || j < 0 || i >= NumRows() || j >= NumCols())
      throw new System.Exception("Matrix index out of bounds.");
    rows[i][j] = val;
  }

  public Matrix(List<T>? rowIn = null)
  {
    if (rowIn != null && rowIn.Count > 0) { rows.Add(new List<T>(rowIn)); }
  }

  public void AddRow(List<T> rowIn)
  {
    if (rows.Count == 0) { rows.Add(new List<T>(rowIn)); return; }
    if (rowIn.Count != rows[0].Count) throw new System.Exception("Rows must have equal length.");
    rows.Add(new List<T>(rowIn));
  }

  private void SwapRows(int i, int j)
  {
    if (i == j) return;
    var tmp = rows[i]; rows[i] = rows[j]; rows[j] = tmp;
  }

  private void ScaleRow(int i, T scal)
  {
    int n = rows[0].Count;
    for (int k = 0; k < n; ++k) { rows[i][k] = rows[i][k].Mul(scal); }
  }

  private void AddScaledRowJtoI(int i, int j, T scal)
  {
    int n = rows[0].Count;
    for (int k = 0; k < n; ++k) { rows[i][k] = rows[i][k].Add(scal.Mul(rows[j][k])); }
  }

  public bool IsSquare()
  {
    if (rows.Count == 0) return true;
    return rows.Count == rows[0].Count;
  }

  private Matrix<T> CloneRows()
  {
    var m = new Matrix<T>();
    foreach (var r in rows) { m.rows.Add(new List<T>(r)); }
    return m;
  }

  public Matrix<T> Rref(out T determinant, out bool linIndep, bool ignoreDeterminant = false)
  {
    determinant = T.FieldZero();
    linIndep = true;
    if (rows.Count == 0) { return new Matrix<T>(); }
    var unit = T.FieldOne();
    var det = T.FieldOne();
    int numR = rows.Count, numC = rows[0].Count;
    var answer = CloneRows();
    bool gotToRref = false;

    while (true)
    {
      if (ignoreDeterminant && !gotToRref)
      {
        answer.rows.Sort((p, q) => getLeadingOneIndex(p).CompareTo(getLeadingOneIndex(q)));
      }

      // Bubble sort (only when tracking determinant)
      if (!ignoreDeterminant && !gotToRref)
      {
        bool swapped = false;
        for (int ii = 0; ii < numR && !swapped; ++ii)
        {
          int leadI = getLeadingOneIndex(answer.rows[ii]);
          for (int jj = ii + 1; jj < numR; ++jj)
          {
            if (leadI <= getLeadingOneIndex(answer.rows[jj])) continue;
            answer.SwapRows(ii, jj);
            det = det.Neg();
            swapped = true;
            break;
          }
        }
        if (swapped) continue;
      }

      // Scale rows to leading 1
      bool adjusting = false;
      if (!gotToRref)
      {
        for (int ii = 0; ii < numR; ++ii)
        {
          int jj = getLeadingOneIndex(answer.rows[ii]);
          if (jj >= numC) { linIndep = false; break; }
          if (answer.rows[ii][jj].FieldEquals(unit)) continue;
          var factor = unit.Div(answer.rows[ii][jj]);
          answer.ScaleRow(ii, factor);
          det = det.Mul(factor);
          adjusting = true;
        }
      }

      // Subtract to zero below leading 1
      if (!gotToRref)
      {
        int prevInd = getLeadingOneIndex(answer.rows[0]);
        for (int ii = 1; ii < numR; ++ii)
        {
          int jj = getLeadingOneIndex(answer.rows[ii]);
          if (jj >= numC) { linIndep = false; break; }
          if (jj > prevInd) { prevInd = jj; continue; }
          answer.AddScaledRowJtoI(ii, prevInd, unit.Neg());
          adjusting = true;
        }
      }

      if (adjusting) continue;

      if (!gotToRref)
      {
        gotToRref = true;
        // Eliminate above leading ones
        for (int ii = numR - 1; ii >= 0; --ii)
        {
          int ind = getLeadingOneIndex(answer.rows[ii]);
          if (ind >= numC) { linIndep = false; continue; }
          for (int jj = ii - 1; jj >= 0; --jj)
          {
            if (answer.rows[jj][ind].IsZero()) continue;
            answer.AddScaledRowJtoI(jj, ii, answer.rows[jj][ind].Neg());
          }
        }
      }
      break;
    }

    if (IsSquare()) { determinant = det; }
    return answer;
  }

  public Matrix<T> Inverse(out bool success)
  {
    var answer = new Matrix<T>();
    if (!IsSquare()) throw new System.Exception("Matrix must be square.");
    int dim = NumRows();
    if (dim == 0) { success = false; return answer; }
    var augmented = new Matrix<T>();
    for (int ii = 0; ii < dim; ++ii)
    {
      var row = new List<T>(rows[ii]);
      for (int jj = 0; jj < dim; ++jj)
      {
        row.Add(ii == jj ? T.FieldOne() : T.FieldZero());
      }
      augmented.AddRow(row);
    }
    T det;
    augmented = augmented.Rref(out det, out success, true);
    if (!success) return answer;
    for (int ii = 0; ii < dim; ++ii)
    {
      var row = new List<T>(dim);
      for (int jj = 0; jj < dim; ++jj) { row.Add(augmented.rows[ii][dim + jj]); }
      answer.AddRow(row);
    }
    return answer;
  }

  public static Matrix<T> operator+(Matrix<T> a, Matrix<T> b)
  {
    if (a.rows.Count == 0 && b.rows.Count == 0) return new Matrix<T>();
    if (a.rows.Count != b.rows.Count || a.rows[0].Count != b.rows[0].Count)
      throw new System.Exception("Matrix sizes must be equal.");
    var ans = new Matrix<T>();
    int nr = a.rows.Count, nc = a.rows[0].Count;
    for (int ii = 0; ii < nr; ++ii)
    {
      var row = new List<T>(nc);
      for (int jj = 0; jj < nc; ++jj) { row.Add(a.rows[ii][jj].Add(b.rows[ii][jj])); }
      ans.rows.Add(row);
    }
    return ans;
  }

  public static Matrix<T> operator-(Matrix<T> a, Matrix<T> b) => a + (-b);

  public static Matrix<T> operator-(Matrix<T> m)
  {
    var ans = new Matrix<T>();
    foreach (var r in m.rows)
    {
      var row = new List<T>(r.Count);
      foreach (var el in r) { row.Add(el.Neg()); }
      ans.rows.Add(row);
    }
    return ans;
  }

  public static Matrix<T> operator*(Matrix<T> m, T scalar)
  {
    var ans = new Matrix<T>();
    foreach (var r in m.rows)
    {
      var row = new List<T>(r.Count);
      foreach (var el in r) { row.Add(el.Mul(scalar)); }
      ans.rows.Add(row);
    }
    return ans;
  }

  public static Matrix<T> operator*(Matrix<T> a, Matrix<T> b)
  {
    int mm = a.rows.Count;
    if (mm == 0) return new Matrix<T>();
    int nn = a.rows[0].Count;
    if (nn != b.rows.Count) throw new System.Exception("Matrix mult: numCols(A) must equal numRows(B).");
    int pp = b.rows[0].Count;
    var ans = new Matrix<T>();
    for (int ii = 0; ii < mm; ++ii)
    {
      var row = new List<T>(pp);
      for (int jj = 0; jj < pp; ++jj)
      {
        var cur = T.FieldZero();
        for (int kk = 0; kk < nn; ++kk) { cur = cur.Add(a.rows[ii][kk].Mul(b.rows[kk][jj])); }
        row.Add(cur);
      }
      ans.rows.Add(row);
    }
    return ans;
  }

  public Matrix<T> Transpose()
  {
    var ans = new Matrix<T>();
    if (rows.Count == 0) return ans;
    int nr = rows.Count, nc = rows[0].Count;
    for (int ii = 0; ii < nc; ++ii)
    {
      var row = new List<T>(nr);
      for (int jj = 0; jj < nr; ++jj) { row.Add(rows[jj][ii]); }
      ans.rows.Add(row);
    }
    return ans;
  }

  public static bool operator==(Matrix<T> a, Matrix<T> b)
  {
    if (a.rows.Count != b.rows.Count) return false;
    for (int ii = 0; ii < a.rows.Count; ++ii)
    {
      if (a.rows[ii].Count != b.rows[ii].Count) return false;
      for (int jj = 0; jj < a.rows[ii].Count; ++jj)
      {
        if (!a.rows[ii][jj].FieldEquals(b.rows[ii][jj])) return false;
      }
    }
    return true;
  }

  public static bool operator!=(Matrix<T> a, Matrix<T> b) => !(a == b);

  public static bool operator<(Matrix<T> a, Matrix<T> b)
  {
    if (a.rows.Count < b.rows.Count) return true;
    if (a.rows.Count > b.rows.Count) return false;
    for (int ii = 0; ii < a.rows.Count; ++ii)
    {
      if (ii == 0)
      {
        if (a.rows[0].Count < b.rows[0].Count) return true;
        if (a.rows[0].Count > b.rows[0].Count) return false;
      }
      for (int jj = 0; jj < a.rows[ii].Count; ++jj)
      {
        int cmp = a.rows[ii][jj].FieldCompareTo(b.rows[ii][jj]);
        if (cmp < 0) return true;
        if (cmp > 0) return false;
      }
    }
    return false;
  }

  public static bool operator>(Matrix<T> a, Matrix<T> b) => b < a;
  public static bool operator<=(Matrix<T> a, Matrix<T> b) => (a < b) || (a == b);
  public static bool operator>=(Matrix<T> a, Matrix<T> b) => (b < a) || (b == a);

  public int CompareTo(Matrix<T>? other)
  {
    if (other is null) return 1;
    if (this < other) return -1;
    if (this == other) return 0;
    return 1;
  }

  public override bool Equals(object? obj)
  {
    if (obj is Matrix<T> other) return this == other;
    return false;
  }

  public override int GetHashCode()
  {
    var h = new HashCode();
    foreach (var r in rows) { foreach (var el in r) { h.Add(el); } }
    return h.ToHashCode();
  }

  public T MatrixDot(Matrix<T> rhs)
  {
    if (rows.Count != rhs.rows.Count || rows[0].Count != rhs.rows[0].Count)
      throw new System.Exception("Matrix sizes must be equal for dot product.");
    var ans = T.FieldZero();
    int nr = rows.Count, nc = rows[0].Count;
    for (int ii = 0; ii < nr; ++ii)
    {
      for (int jj = 0; jj < nc; ++jj)
      {
        ans = ans.Add(rows[ii][jj].Mul(rhs.rows[ii][jj]));
      }
    }
    return ans;
  }

  public T MatrixSqNorm() => MatrixDot(this);

  public static Matrix<T> Cross(Matrix<T> u, Matrix<T> v)
  {
    if (u.NumRows() != 3 || u.NumCols() != 1 || v.NumRows() != 3 || v.NumCols() != 1)
      throw new System.Exception("Cross product requires 3x1 column vectors.");
    var ans = ZeroMatrix(3, 1);
    ans.rows[0][0] = u.At(1,0).Mul(v.At(2,0)).Sub(v.At(1,0).Mul(u.At(2,0)));
    ans.rows[1][0] = u.At(2,0).Mul(v.At(0,0)).Sub(v.At(2,0).Mul(u.At(0,0)));
    ans.rows[2][0] = u.At(0,0).Mul(v.At(1,0)).Sub(v.At(0,0).Mul(u.At(1,0)));
    return ans;
  }

  public static Matrix<T> GetRotation(Matrix<T> vecFrom, Matrix<T> vecTo)
  {
    int dim = vecFrom.NumRows();
    var ans = ZeroMatrix(dim);
    if (vecTo.NumCols() != 1 || vecFrom.NumCols() != 1 || vecTo.NumRows() != vecFrom.NumRows())
      throw new System.Exception("vecFrom and vecTo must be column vectors of equal length.");
    var u2 = vecFrom.MatrixSqNorm();
    var v2 = vecTo.MatrixSqNorm();
    var zero_ = u2.Sub(u2);
    if (u2.FieldEquals(zero_) || v2.FieldEquals(zero_))
      throw new System.Exception("vecFrom and vecTo must be non-zero.");
    var one_ = u2.Div(u2);
    if (!u2.FieldEquals(one_) || !v2.FieldEquals(one_))
      throw new System.Exception("vecFrom and vecTo must have unit length.");
    if (dim == 2)
    {
      var x0 = vecFrom.At(0,0); var y0 = vecFrom.At(1,0);
      var x1 = vecTo.At(0,0);   var y1 = vecTo.At(1,0);
      var aa = x0.Mul(x1).Add(y0.Mul(y1));
      var bb = y0.Mul(x1).Sub(x0.Mul(y1));
      ans.rows[0][0] = aa;       ans.rows[0][1] = bb;
      ans.rows[1][0] = bb.Neg(); ans.rows[1][1] = aa;
      return ans;
    }
    if (dim != 3) throw new System.Exception("Inputs must be 2-vectors or 3-vectors.");
    var II = Identity(3);
    var uXv = Cross(vecFrom, vecTo);
    var sk = ZeroMatrix(3);
    sk.rows[0][1] = uXv.At(2,0).Neg(); sk.rows[0][2] = uXv.At(1,0);
    sk.rows[1][0] = uXv.At(2,0);       sk.rows[1][2] = uXv.At(0,0).Neg();
    sk.rows[2][0] = uXv.At(1,0).Neg(); sk.rows[2][1] = uXv.At(0,0);
    var K2 = sk * sk * one_.Div(uXv.MatrixSqNorm());
    var omc = one_.Sub(vecFrom.MatrixDot(vecTo));
    ans = II + sk + K2 * omc;
    if (ans * vecFrom != vecTo) { ans = II + (-sk) + K2 * omc; }
    if (ans * vecFrom != vecTo)
      throw new System.Exception("Matrix does not map vecFrom to vecTo.");
    if (ans * ans.Transpose() != II)
      throw new System.Exception("Matrix is not a rotation.");
    return ans;
  }
}
}
