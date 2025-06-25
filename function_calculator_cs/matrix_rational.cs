/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

namespace function_calculator_cs
{

public class MatrixRational
{
  private List<List<Rational> > rows = new List<List<Rational> >();

  private static int getLeadingOneIndex(in List<Rational> row)
  {
    Rational zero_ = new Rational();
    int num_Cols = (int)row.Count;
    int ii = 0;
    for (; ii < num_Cols; ++ii)
    {
      if (row[ii] != zero_) { return ii; }
    }
    return ii;
  }

  /** \brief Useful for resorting rows in MatrixRational. */
  private static int compareLessThan(in List<Rational> P, in List<Rational> Q)
  {
    if (getLeadingOneIndex(P) < getLeadingOneIndex(Q)) { return 1; }
    return -1;
  }

  public override string ToString()
  {
    return ToString(false);
  }

  public string ToString(Boolean useParentheses)
  {
    if (rows.Count == 0) { return (useParentheses ? "(0)" : "0"); }
    string strm = "";
    int num_Rows = (int)rows.Count;
    int num_Cols = (int)rows[0].Count;
    List<List<string>> buffer = new List<List<string>>();
    List<int> longestRows = new List<int>();
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      buffer.Add(new List<string>());
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        buffer[ii].Add("");
      }
    }
    for (int ii = 0; ii < num_Cols; ++ii)
    {
      longestRows.Add(0);
    }
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        string current = "";
        current += rows[ii][jj];
        buffer[ii][jj] = current;
        if ((int)(current.Length) > longestRows[jj]) { longestRows[jj] = (int)(current.Length); }
      }
    }
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      strm += "\n";
      if (useParentheses)
      {
        if (num_Rows == 1) { strm += "("; } else { strm += "|| "; }
      }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        if (jj > 0)
        {
          if (num_Rows == 1) { strm += ", "; } else { strm += " | "; }
        }
        var current = buffer[ii][jj];
        int currentLength = (int)current.Length;
        strm += current; // TODO: Deal with situation where entries themselves have multiple rows.
        for (int kk = 0; kk < (longestRows[jj] - currentLength); ++kk) { strm += " "; }
      }
      if (useParentheses)
      {
        if (num_Rows == 1) { strm += ")"; } else { strm += " ||"; }
      }
    }
    return strm;
  }

  public int numRows() { return (int)(rows.Count); }
  public int numCols()
  {
    if (rows.Count == 0) { return 0; }
    return (int)(rows[0].Count);
  }

  public static MatrixRational zeroMatrix(int rowDim, int colDim)
  {
    MatrixRational answer = new MatrixRational();
    List<Rational> row = new List<Rational>();
    for (int ii = 0; ii < colDim; ++ii) { row.Add(new Rational()); }
    for (int ii = 0; ii < rowDim; ++ii) { answer.addRow(row); }
    return answer;
  }

  public static MatrixRational zeroMatrix(int dim)
  {
    return zeroMatrix(dim, dim);
  }

  public static MatrixRational identity(int dim)
  {
    MatrixRational answer = new MatrixRational();
    for (int ii = 0; ii < dim; ++ii)
    {
      List<Rational> row = new List<Rational>();
      for (int jj = 0; jj < dim; ++jj)
      {
        row.Add(new Rational());
      }
      row[ii] = new Rational(1);
      answer.addRow(row);
    }
    return answer;
  }

  public Rational at(int i, int j)
  {
    if (i < 0) { throw new System.Exception("Row index cannot be negative."); }
    if (j < 0) { throw new System.Exception("Columns index cannot be negative."); }
    if (i >= numRows()) { throw new System.Exception("Row index must be less than MatrixRational dimension."); }
    if (j >= numCols()) { throw new System.Exception("Columns index must be less than MatrixRational dimension."); }
    return new Rational(rows[i][j]);
  }

  public MatrixRational()
  {
    rows = new List<List<Rational> >();
  }

  public MatrixRational(in List<Rational> rowIn)
  {
    rows = new List<List<Rational> >();
    if (rowIn.Count > 0) { rows.Add(new List<Rational>(rowIn)); }
  }

  public MatrixRational(in MatrixRational rhs)
  {
    rows = new List<List<Rational> >();
    if (!(rhs is null))
    {
      for (int ii = 0; ii < rhs.rows.Count; ++ii)
      {
        var current = new List<Rational>();
        for (int jj = 0; jj < rhs.rows[ii].Count; ++jj)
        {
          current.Add(new Rational(rhs.rows[ii][jj]));
        }
        rows.Add(current);
      }
    }
  }

  public void addRow(in List<Rational> rowIn)
  {
    if (rows.Count == 0) { rows.Add(new List<Rational>(rowIn)); return; }
    if (rowIn.Count != rows[0].Count) { throw new System.Exception("Rows must have equal length."); }
    rows.Add(new List<Rational>(rowIn));
  }

  /** \brief Elementary row operation. Changes sign of the determinant if i != j. */
  public void swapRows(int i, int j)
  {
    if (i < 0) { throw new System.Exception("Index out of bounds."); } // If size == 0 one of these always is called.
    if (j < 0) { throw new System.Exception("Index out of bounds."); }
    if (i >= ((int)rows.Count)) { throw new System.Exception("Index out of bounds."); }
    if (j >= ((int)rows.Count)) { throw new System.Exception("Index out of bounds."); }
    if (i == j) { return; }
    int nn = (int)(rows[0].Count);
    for (int kk = 0; kk < nn; ++kk)
    {
      var temp = new Rational(rows[i][kk]);
      rows[i][kk] = rows[j][kk];
      rows[j][kk] = temp;
    }
  }

  /** \brief Elementary row operation. Multiplies the determinant by scal. */
  public void scaleRow(int i, in Rational scal)
  {
    if (i < 0) { throw new System.Exception("Index out of bounds."); } // If size == 0 one of these always is called.
    if (i >= ((int)rows.Count)) { throw new System.Exception("Index out of bounds."); }
    int nn = (int)(rows[0].Count);
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = scal * rows[i][kk];
    }
  }

  /** \brief Elementary row operation. Leaves determinant unchanged if i != j, otherwise acts as scaleRow method by (1 + scal). */
  public void addScaledRowJ_toI(int i, int j, in Rational scal)
  {
    if (i < 0) { throw new System.Exception("Index out of bounds."); } // If size == 0 one of these always is called.
    if (j < 0) { throw new System.Exception("Index out of bounds."); }
    if (i >= ((int)rows.Count)) { throw new System.Exception("Index out of bounds."); }
    if (j >= ((int)rows.Count)) { throw new System.Exception("Index out of bounds."); }
    int nn = (int)(rows[0].Count);
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = rows[i][kk] + scal * rows[j][kk];
    }
  }

  public Boolean isSquare()
  {
    if (rows.Count == 0) { return true; }
    return (rows.Count == (rows[0].Count));
  }

  /** \return Reduced row echelon form.
   * 
   *  \param `determinant` Output reference is set to the determinant of the MatrixRational, or 0 if MatrixRational is not square.
   *  \param `linInd` Output reference is set to true if rows of the MatrixRational are linearly independent
   *  \param `ignoreDeterminant` Speeds up algorithm by ignoring determinant (use O(n * log(n)) sort rather than bubble sort)
   */
  public MatrixRational rref(ref Rational determinant, ref Boolean linIndep, Boolean ignoreDeterminant = false)
  {
    if (!ignoreDeterminant) { determinant = new Rational(); }
    linIndep = true;
    if (rows.Count == 0) { return new MatrixRational(this); }
    Rational unit = new Rational(1);
    Rational det = new Rational(unit);
    int num_Rows = (int)rows.Count;
    int num_Cols = (int)rows[0].Count;
    MatrixRational answer = new MatrixRational(this);
    bool gotToRowEchelon = false;
    for (bool performingRref = true; performingRref; performingRref = !performingRref)
    {
      //if (ignoreDeterminant && (!gotToRowEchelon)) // Rearrange rows...
      {
        //answer.rows.Sort((P, Q) => compareLessThan(P, Q)); // Sort in ascending order.
      }
      // else ... Bubble sort while computing determinant.
      for (int ii = 0; ii < num_Rows; ++ii)
      {
        //if (ignoreDeterminant) { break; }
        if (gotToRowEchelon) { break; }
        int leadI = getLeadingOneIndex(answer.rows[ii]);
        for (int jj = ii + 1; jj < num_Rows; ++jj)
        {
          int leadJ = getLeadingOneIndex(answer.rows[jj]);
          if (leadI <= leadJ) { continue; }
          answer.swapRows(ii, jj);
          det = -det;
          performingRref = false;
          break;
        }
        if (!performingRref) { break; }
      }
      if (!performingRref) { continue; }
      Boolean adjusting = false;
      for (int ii = 0; ii < num_Rows; ++ii) // Divide by leading coefficient.
      {
        if (gotToRowEchelon) { break; }
        int jj = getLeadingOneIndex(answer.rows[ii]);
        if (jj >= num_Cols) { linIndep = false; break; }
        Rational leadCoeff = new Rational(answer.rows[ii][jj]);
        if (leadCoeff == unit) { continue; }
        var factor = unit / leadCoeff;
        answer.scaleRow(ii, factor);
        det = det * factor;
        adjusting = true;
      }
      // Subtract rows where applicable.
      {
        int prevInd = -1;
        if (!gotToRowEchelon) { prevInd = getLeadingOneIndex(answer.rows[0]); }
        for (int ii = 1; ii < num_Rows; ++ii)
        {
          if (gotToRowEchelon) { break; }
          int jj = getLeadingOneIndex(answer.rows[ii]);
          if (jj >= num_Cols) { linIndep = false; break; }
          if (jj > prevInd) { prevInd = jj; continue; }
          answer.addScaledRowJ_toI(ii, prevInd, -unit); // det unchanged.
          adjusting = true;
        }
      }
      if (adjusting) { performingRref = false; continue; }
      gotToRowEchelon = true;
      // Now from row echelon form, modify to reduced row echelon form.
      for (int ii = num_Rows - 1; ii >= 0; --ii)
      {
        int ind = getLeadingOneIndex(answer.rows[ii]);
        if (ind >= num_Cols) { linIndep = false; continue; }
        for (int jj = ii - 1; jj >= 0; --jj)
        {
          if (answer.rows[jj][ind] == new Rational()) { continue; }
          answer.addScaledRowJ_toI(jj, ii, -(answer.rows[jj][ind])); // det unchanged.
        }
      }
      // Now the answer is in rref.
    }
    if (isSquare()) { determinant = new Rational(det); }
    return answer;
  }

  public Rational determinant()
  {
    Rational det = new Rational(); bool success = false;
    rref(ref det, ref success);
    return det;
  }

  /** \param `success` is true if and only MatrixRational is invertible.
   *  \return Inverse MatrixRational if the MatrixRational is invertible, zero otherwise.
   */
  public MatrixRational inverse(ref Boolean success)
  {
    MatrixRational answer = new MatrixRational();
    if (!isSquare()) { success = false; throw new System.Exception("MatrixRational must be square."); }
    int dim = numRows();
    if (dim == 0) { success = false; return answer; }
    Rational zero_ = new Rational();
    Rational unit = new Rational(1);
    MatrixRational rREFed = new MatrixRational();
    for (int ii = 0; ii < dim; ++ii)
    {
      var newRow = new List<Rational>(rows[ii]);
      {
        int newRowCount = newRow.Count;
        for (int jj = newRowCount; jj < 2 * dim; ++jj)
        {
          newRow.Add(new Rational());
        }
      }
      for (int jj = 0; jj < dim; ++jj) { newRow[dim + jj] = ((ii == jj) ? unit : zero_); }
      rREFed.addRow(newRow);
    }
    Rational det = new Rational();
    rREFed = rREFed.rref(ref det, ref success, true);
    if (!success) { return answer; }
    var dimTwice = 2 * dim;
    for (int ii = 0; ii < dim; ++ii)
    {
      var newRow = new List<Rational>();
      for (int jj = 0; jj < dim; ++jj) { newRow.Add(new Rational()); }
      var current = rREFed.rows[ii];
      for (int jj = 0; jj < dim; ++jj)
      {
        newRow[jj] = new Rational(current[dim + jj]);
      }
      answer.addRow(newRow);
    }
    return answer;
  }

  public static MatrixRational operator+(in MatrixRational body) { return body; }
  public static MatrixRational operator-(in MatrixRational body)
  {
    MatrixRational answer = new MatrixRational(body);
    for (int ii = 0; ii < answer.rows.Count; ++ii)
    {
      for (int jj = 0; jj < answer.rows[ii].Count; ++jj)
      {
        answer.rows[ii][jj] = -answer.rows[ii][jj];
      }
    }
    return answer;
  }

  public static MatrixRational operator+(in MatrixRational body, in MatrixRational rhs)
  {
    if ((body.rows.Count == 0) && !(rhs.rows.Count == 0)) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if ((body.rows.Count == 0) && (rhs.rows.Count == 0)) { return new MatrixRational(); }
    if ((rhs.rows.Count == 0) && !(body.rows.Count == 0)) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if (body.rows.Count != rhs.rows.Count) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if (body.rows[0].Count != rhs.rows[0].Count) { throw new System.Exception("MatrixRational sizes must be equal."); }
    int num_Rows = (int)body.rows.Count; int num_Cols = (int)body.rows[0].Count;
    MatrixRational answer = new MatrixRational(body);
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] + rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  public static MatrixRational operator-(in MatrixRational body, in MatrixRational rhs) { return (body + (-rhs)); }

  public static MatrixRational operator*(in MatrixRational body, in Rational rhs)
  {
    MatrixRational answer = new MatrixRational(body);
    int num_Rows = body.numRows();
    if (num_Rows == 0) { return answer; }
    int num_Cols = body.numCols();
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] * rhs;
      }
    }
    return answer;
  }

  public static MatrixRational operator*(in MatrixRational body, in MatrixRational rhs)
  {
    MatrixRational answer = new MatrixRational();

    int mm = (int)body.rows.Count;
    if (mm == 0) { return answer; }
    int nn = (int)body.rows[0].Count;
    if (nn != (int)rhs.rows.Count) { throw new System.Exception("Matrix mult A * B: numCols(A) must equal numRows(B)"); }
    int pp = (int)rhs.rows[0].Count;

    answer = zeroMatrix(mm, pp);

    for (int ii = 0; ii < mm; ++ii)
    {
      for (int jj = 0; jj < pp; ++jj)
      {
        for (int kk = 0; kk < nn; ++kk)
        {
          answer.rows[ii][jj] = answer.rows[ii][jj] + body.rows[ii][kk] * rhs.rows[kk][jj];
        }
      }
    }
    return answer;
  }

  public MatrixRational transpose()
  {
    MatrixRational answer = new MatrixRational();
    if (rows.Count == 0) { return answer; }

    int num_Rows = (int)rows.Count;
    int num_Cols = (int)rows[0].Count;
    answer = zeroMatrix(num_Cols, num_Rows);

    for (int ii = 0; ii < num_Cols; ++ii)
    {
      for (int jj = 0; jj < num_Rows; ++jj)
      {
        answer.rows[ii][jj] = new Rational(rows[jj][ii]);
      }
    }

    return answer;
  }

  public override bool Equals(object? obj)
  {
    return Equals(obj as MatrixRational);
  }

  public static Boolean operator==(in MatrixRational body, in MatrixRational rhs)
  {
    return body.Equals(rhs);
  }

  public bool Equals(MatrixRational? other)
  {
    if (other is null) { return false; }
    if (rows.Count != other.rows.Count) { return false; }
    int num_Rows = (int)rows.Count;
    int num_Cols = 0;
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      if (rows[ii].Count != other.rows[ii].Count) { return false; }
      if (ii > 0)
      {
        if (rows[0].Count != rows[ii].Count) { throw new System.Exception("Matrices must have the same size."); }
        if (other.rows[0].Count != other.rows[ii].Count) { throw new System.Exception("Matrices must have the same size."); }
      }
      else
      { num_Cols = (int)rows[0].Count; }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        if (rows[ii][jj] == other.rows[ii][jj]) { continue; }
        return false;
      }
    }
    return true;
  }

  public static bool operator!=(in MatrixRational body, in MatrixRational rhs)
  {
    return !(body == rhs);
  }

  public static bool operator<(in MatrixRational body, in MatrixRational rhs)
  {
    if (body.rows.Count < rhs.rows.Count) { return true; }
    if (body.rows.Count > rhs.rows.Count) { return false; }
    int num_Rows = (int)body.rows.Count;
    int num_Cols = 0;
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      if (ii > 0)
      {
        if (body.rows[0].Count != body.rows[ii].Count) { throw new System.Exception("Matrices must have the same size."); }
        if (rhs.rows[0].Count != rhs.rows[ii].Count) { throw new System.Exception("Matrices must have the same size."); }
      }
      else
      {
        if (body.rows[0].Count < rhs.rows[0].Count) { return true; }
        if (body.rows[0].Count > rhs.rows[0].Count) { return false; }
        num_Cols = (int)body.rows[0].Count;
      }
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        var aa = body.rows[ii][jj];
        var bb = rhs.rows[ii][jj];
        if (aa < bb) { return true; }
        if (aa == bb) { continue; }
        return false;
      }
    }
    return false;
  }

  public static bool operator>(in MatrixRational body, in MatrixRational rhs)
  {
    return (rhs < body);
  }

  public static bool operator<=(in MatrixRational body, in MatrixRational rhs)
  {
    if (body < rhs) { return true; }
    if (body == rhs) { return true; }
    return false;
  }

  public static bool operator>=(in MatrixRational body, in MatrixRational rhs)
  {
    if (rhs < body) { return true; }
    if (rhs == body) { return true; }
    return false;
  }

  public Rational MatrixDot(in MatrixRational rhs)
  {
    Rational answer = new Rational();
    if ((rows.Count == 0) && !(rhs.rows.Count == 0)) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if ((rhs.rows.Count == 0) && !(rows.Count == 0)) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if (rows.Count != rhs.rows.Count) { throw new System.Exception("MatrixRational sizes must be equal."); }
    if (rows[0].Count != rhs.rows[0].Count) { throw new System.Exception("MatrixRational sizes must be equal."); }
    int num_Rows = (int)rows.Count; int num_Cols = (int)rows[0].Count;
    if ((num_Rows == 0) || (num_Cols == 0)) { return answer; }
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        answer = answer + rows[ii][jj] * rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  public Rational MatrixSqNorm() { return MatrixDot(this); }

  public override int GetHashCode()
  {
    var hash = new HashCode();
    int num_Rows = numRows(); int num_Cols = numCols();
    for (int ii = 0; ii < num_Rows; ++ii)
    {
      for (int jj = 0; jj < num_Cols; ++jj)
      {
        hash.Add(new Rational(rows[ii][jj]));
      }
    }
    return hash.ToHashCode();
  }
}
}

