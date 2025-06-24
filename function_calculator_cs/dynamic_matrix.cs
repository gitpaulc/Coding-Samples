/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

using function_calculator_cs;

namespace function_calculator_cs
{

public class MatrixRational
{
  private List<List<Rational> > rows = new List<List<Rational> >();

  private static int getLeadingOneIndex(in List<Rational> row)
  {
    Rational zero_ = new Rational();
    int Rational_Cols = (int)row.Count;
    int ii = 0;
    for (; ii < Rational_Cols; ++ii)
    {
      if (row[ii] != zero_) { return ii; }
    }
    return ii;
  }

  /** \brief Useful for resorting rows in MatrixRational. */
  private static Boolean compareLessThan(in List<Rational> P, in List<Rational> Q)
  {
    return getLeadingOneIndex(P) < getLeadingOneIndex(Q);
  }

  public override string ToString()
  {
    return ToString(false);
  }

  public string ToString(Boolean useParentheses)
  {
    if (rows.Count == 0) { return (useParentheses ? "(0)" : "0"); }
    string strm = "";
    int Rational_Rows = (int)rows.Count;
    int Rational_Cols = (int)rows[0].Count;
    List<List<string>> buffer = new List<List<string>>();
    List<int> longestRows = new List<int>();
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      buffer.Add(new List<string>());
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        buffer[ii].Add("");
      }
    }
    for (int ii = 0; ii < Rational_Cols; ++ii)
    {
      longestRows.Add(0);
    }
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        string current = "";
        current += rows[ii][jj];
        buffer[ii][jj] = current;
        if ((int)(current.Length) > longestRows[jj]) { longestRows[jj] = (int)(current.Length); }
      }
    }
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      strm += "\n";
      if (useParentheses)
      {
        if (Rational_Rows == 1) { strm += "("; } else { strm += "|| "; }
      }
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        if (jj > 0)
        {
          if (Rational_Rows == 1) { strm += ", "; } else { strm += " | "; }
        }
        var current = buffer[ii][jj];
        int currentLength = (int)current.Length;
        strm += current; // TODO: Deal with situation where entries themselves have multiple rows.
        for (int kk = 0; kk < (longestRows[jj] - currentLength); ++kk) { strm += " "; }
      }
      if (useParentheses)
      {
        if (Rational_Rows == 1) { strm += ")"; } else { strm += " ||"; }
      }
    }
    return strm;
  }

  public int RationalRows() { return (int)(rows.Count); }
  public int RationalCols()
  {
    if (rows.Count == 0) { return 0; }
    return (int)(rows[0].Count);
  }

  public static MatrixRational zeroMatrixRational(int rowDim, int colDim)
  {
    MatrixRational answer = new MatrixRational();
    List<Rational> row = new List<Rational>();
    for (int ii = 0; ii < colDim; ++ii) { row.Add(new Rational()); }
    for (int ii = 0; ii < rowDim; ++ii) { answer.addRow(row); }
    return answer;
  }

  public static MatrixRational zeroMatrixRational(int dim)
  {
    return zeroMatrixRational(dim, dim);
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
    if (i >= RationalRows()) { throw new System.Exception("Row index must be less than MatrixRational dimension."); }
    if (j >= RationalCols()) { throw new System.Exception("Columns index must be less than MatrixRational dimension."); }
    return new Rational(rows[i][j]);
  }

  public MatrixRational()
  {
    rows = new List<List<Rational> >();
  }

  public MatrixRational(in List<Rational> rowIn)
  {
    rows = new List<List<Rational> >();
    if (rowIn.Count > 0) { rows.Add(rowIn); }
  }

  public void addRow(in List<Rational> rowIn)
  {
    if (rows.Count == 0) { rows.Add(rowIn); return; }
    if (rowIn.Count != rows[0].Count) { throw new System.Exception("Rows must have equal length."); }
    rows.Add(rowIn);
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
  public void scaleRow(int i, const Rational& scal)
  {
    if (i < 0) { throw std::invalid_argument("Index out of bounds."); return; } // If size == 0 one of these always is called.
    if (i >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    int nn = (int)(rows[0].size());
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = scal * rows[i][kk];
    }
  }

  /** \brief Elementary row operation. Leaves determinant unchanged if i != j, otherwise acts as scaleRow method by (1 + scal). */
  public void addScaledRowJ_toI(int i, int j, const Rational& scal)
  {
    if (i < 0) { throw std::invalid_argument("Index out of bounds."); return; } // If size == 0 one of these always is called.
    if (j < 0) { throw std::invalid_argument("Index out of bounds."); return; }
    if (i >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    if (j >= ((int)rows.size())) { throw std::invalid_argument("Index out of bounds."); return; }
    int nn = (int)(rows[0].size());
    for (int kk = 0; kk < nn; ++kk)
    {
      rows[i][kk] = rows[i][kk] + scal * rows[j][kk];
    }
  }

  public Boolean isSquare()
  {
    if (rows.Count == 0) { return true; }
    return (rows.Count == (rows[0].size()));
  }

  /** \return Reduced row echelon form.
   * 
   *  \param `determinant` Output reference is set to the determinant of the MatrixRational, or 0 if MatrixRational is not square.
   *  \param `linInd` Output reference is set to true if rows of the MatrixRational are linearly independent
   *  \param `ignoreDeterminant` Speeds up algorithm by ignoring determinant (use O(n * log(n)) sort rather than bubble sort)
   */
  public MatrixRational rref(ref Rational determinant, ref Boolean linIndep, Boolean ignoreDeterminant = false)
  {
    if (!ignoreDeterminant) { determinant = Rational(); }
    linIndep = true;
    if (rows.empty()) { return *this; }
    Rational unit(1);
    Rational det = unit;
    int Rational_Rows = (int)rows.size();
    int Rational_Cols = (int)rows[0].size();
    MatrixRational answer;
    answer.rows = rows;
    bool gotToRowEchelon = false;
    for (bool performingRref = true; performingRref; performingRref = !performingRref)
    {
      if (ignoreDeterminant && (!gotToRowEchelon)) // Rearrange rows...
      {
        std::sort(answer.rows.begin(), answer.rows.end(), compareLessThan);
      }
      // else ... Bubble sort while computing determinant.
      for (int ii = 0; ii < Rational_Rows; ++ii)
      {
        if (ignoreDeterminant) { break; }
        if (gotToRowEchelon) { break; }
        int leadI = getLeadingOneIndex(answer.rows[ii]);
        for (int jj = ii + 1; jj < Rational_Rows; ++jj)
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
      bool adjusting = false;
      for (int ii = 0; ii < Rational_Rows; ++ii) // Divide by leading coefficient.
      {
        if (gotToRowEchelon) { break; }
        int jj = getLeadingOneIndex(answer.rows[ii]);
        if (jj >= Rational_Cols) { linIndep = false; break; }
        if (answer.rows[ii][jj] == unit) { continue; }
        auto factor = unit / answer.rows[ii][jj];
        answer.scaleRow(ii, factor);
        det = det * factor;
        adjusting = true;
      }
      // Subtract rows where applicable.
      {
        int prevInd = -1;
        if (!gotToRowEchelon) { prevInd = getLeadingOneIndex(answer.rows[0]); }
        for (int ii = 1; ii < Rational_Rows; ++ii)
        {
          if (gotToRowEchelon) { break; }
          int jj = getLeadingOneIndex(answer.rows[ii]);
          if (jj >= Rational_Cols) { linIndep = false; break; }
          if (jj > prevInd) { prevInd = jj; continue; }
          answer.addScaledRowJ_toI(ii, prevInd, -unit); // det unchanged.
          adjusting = true;
        }
      }
      if (adjusting) { performingRref = false; continue; }
      gotToRowEchelon = true;
      // Now from row echelon form, modify to reduced row echelon form.
      for (int ii = Rational_Rows - 1; ii >= 0; --ii)
      {
        int ind = getLeadingOneIndex(answer.rows[ii]);
        if (ind >= Rational_Cols) { linIndep = false; continue; }
        for (int jj = ii - 1; jj >= 0; --jj)
        {
          if (answer.rows[jj][ind] == Rational()) { continue; }
          answer.addScaledRowJ_toI(jj, ii, -(answer.rows[jj][ind])); // det unchanged.
        }
      }
      // Now the answer is in rref.
    }
    if (isSquare()) { determinant = det; }
    return answer;
  }

  public Rational determinant()
  {
    Rational det; bool success = false;
    rref(det, success);
    return det;
  }

  /** \param `success` is true if and only MatrixRational is invertible.
   *  \return Inverse MatrixRational if the MatrixRational is invertible, zero otherwise.
   */
  public MatrixRational inverse(bool& success) const
  {
    MatrixRational answer;
    if (!isSquare()) { throw std::invalid_argument("MatrixRational must be square."); success = false; return answer; }
    const int dim = RationalRows();
    if (dim == 0) { success = false; return answer; }
    Rational zero;
    Rational unit(1);
    MatrixRational rREFed;
    for (int ii = 0; ii < dim; ++ii)
    {
      auto newRow = rows[ii];
      newRow.resize(2 * dim);
      for (int jj = 0; jj < dim; ++jj) { newRow[dim + jj] = ((ii == jj) ? unit : zero); }
      rREFed.addRow(newRow);
    }
    Rational det;
    rREFed = rREFed.rref(det, success, true);
    if (!success) { return answer; }
    auto dimTwice = 2 * dim;
    for (int ii = 0; ii < dim; ++ii)
    {
      std::vector<Rational> newRow(dim);
      auto& current = rREFed.rows[ii];
      for (int jj = 0; jj < dim; ++jj)
      {
        newRow[jj] = current[dim + jj];
      }
      answer.addRow(newRow);
    }
    return answer;
  }

  public static MatrixRational operator+(in MatrixRational body) { return body; }
  MatrixRational operator-() const
  {
    MatrixRational answer;
    answer.rows = rows;
    for (auto& row : rows) { for (auto& item : row) { item = -item; } }
    return answer;
  }

  MatrixRational operator+(const MatrixRational& rhs) const
  {
    MatrixRational answer;
    if (rows.empty() && !(rhs.rows.empty())) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rhs.rows.empty() && !(rows.empty())) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rows.size() != rhs.rows.size()) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rows[0].size() != rhs.rows[0].size()) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    int Rational_Rows = (int)rows.size(); int Rational_Cols = (int)rows[0].size();
    answer.rows = rows;
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] + rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  MatrixRational operator-(const MatrixRational& rhs) const { return ((*this) + (-rhs)); }

  MatrixRational operator*(const Rational& rhs) const
  {
    MatrixRational answer;
    answer.rows = rows;
    const int Rational_Rows = RationalRows();
    if (Rational_Rows == 0) { return answer; }
    const int Rational_Cols = RationalCols();
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        answer.rows[ii][jj] = answer.rows[ii][jj] * rhs;
      }
    }
    return answer;
  }

  MatrixRational operator*(const MatrixRational& rhs) const
  {
    MatrixRational answer;

    int mm = (int)rows.size();
    if (mm == 0) { return answer; }
    int nn = (int)rows[0].size();
    if (nn != (int)rhs.rows.size()) { throw std::invalid_argument("MatrixRational mult A * B: RationalCols(A) must equal RationalRows(B)"); return answer; }
    int pp = (int)rhs.rows[0].size();

    answer.rows.resize(mm);
    for (int ii = 0; ii < mm; ++ii) { answer.rows[ii].resize(pp); }

    for (int ii = 0; ii < mm; ++ii)
    {
      for (int jj = 0; jj < pp; ++jj)
      {
        auto& current = answer.rows[ii][jj];
        for (int kk = 0; kk < nn; ++kk)
        {
          current = current + rows[ii][kk] * rhs.rows[kk][jj];
        }
      }
    }
    return answer;
  }

  MatrixRational transpose() const
  {
    MatrixRational answer;
    if (rows.empty()) { return answer; }

    int Rational_Rows = (int)rows.size();
    int Rational_Cols = (int)rows[0].size();

    answer.rows.resize(Rational_Cols);
    for (int ii = 0; ii < Rational_Cols; ++ii) { answer.rows[ii].resize(Rational_Rows); }

    for (int ii = 0; ii < Rational_Cols; ++ii) { for (int jj = 0; jj < Rational_Rows; ++jj) { answer.rows[ii][jj] = rows[jj][ii]; } }
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
    if (rows.size() != rhs.rows.size()) { return false; }
    int Rational_Rows = (int)rows.size();
    int Rational_Cols = 0;
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      if (rows[ii].size() != rhs.rows[ii].size()) { return false; }
      if (ii > 0)
      {
        if (rows[0].size() != rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
        if (rhs.rows[0].size() != rhs.rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
      }
      else
      { Rational_Cols = (int)rows[0].size(); }
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        if (rows[ii][jj] == rhs.rows[ii][jj]) { continue; }
        return false;
      }
    }
    return true;
  }

  bool operator!=(const MatrixRational& rhs) const
  {
    return !((*this) == rhs);
  }

  bool operator<(const MatrixRational& rhs) const
  {
    if (rows.size() < rhs.rows.size()) { return true; }
    if (rows.size() > rhs.rows.size()) { return false; }
    int Rational_Rows = (int)rows.size();
    int Rational_Cols = 0;
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      if (ii > 0)
      {
        if (rows[0].size() != rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
        if (rhs.rows[0].size() != rhs.rows[ii].size()) { throw std::invalid_argument("Matrices must have the same size."); }
      }
      else
      {
        if (rows[0].size() < rhs.rows[0].size()) { return true; }
        if (rows[0].size() > rhs.rows[0].size()) { return false; }
        Rational_Cols = (int)rows[0].size();
      }
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        auto& aa = rows[ii][jj];
        auto& bb = rhs.rows[ii][jj];
        if (aa < bb) { return true; }
        if (aa == bb) { continue; }
        return false;
      }
    }
    return false;
  }

  bool operator>(const MatrixRational& rhs) const
  {
    return (rhs < (*this));
  }

  bool operator<=(const MatrixRational& rhs) const
  {
    if ((*this) < rhs) { return true; }
    if ((*this) == rhs) { return true; }
    return false;
  }

  bool operator>=(const MatrixRational& rhs) const
  {
    if (rhs < (*this)) { return true; }
    if (rhs == (*this)) { return true; }
    return false;
  }

  Rational MatrixRationalDot(const MatrixRational& rhs) const
  {
    Rational answer;
    answer = answer - answer;
    if (rows.empty() && !(rhs.rows.empty())) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rhs.rows.empty() && !(rows.empty())) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rows.size() != rhs.rows.size()) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    if (rows[0].size() != rhs.rows[0].size()) { throw std::invalid_argument("MatrixRational sizes must be equal."); return answer; }
    int Rational_Rows = (int)rows.size(); int Rational_Cols = (int)rows[0].size();
    if ((Rational_Rows == 0) || (Rational_Cols == 0)) { return answer; }
    for (int ii = 0; ii < Rational_Rows; ++ii)
    {
      for (int jj = 0; jj < Rational_Cols; ++jj)
      {
        answer = answer + rows[ii][jj] * rhs.rows[ii][jj];
      }
    }
    return answer;
  }

  Rational MatrixRationalSqNorm() const { return MatrixRationalDot(*this); }

  public override int GetHashCode()
  {
    return HashCode.Combine(rows);
  }
}
}

