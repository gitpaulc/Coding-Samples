namespace function_calculator_cs
{
  public partial class MainWindow : Form
  {

    Boolean test_mp()
    {
      console.Text = "";
      mp zero = mp.zero();
      console.Text += "Testing multiprecision integers:";
      console.Text += Environment.NewLine + Environment.NewLine + "0 = " + zero;
      mp one = new mp(1);
      mp two = one + one;
      console.Text += Environment.NewLine + "1 = " + one + Environment.NewLine + "2 = " + two;
      mp twoToThe16 = two.pow(16);
      console.Text += Environment.NewLine + "65,536 = " + twoToThe16;
      mp thousand = new mp(1000);
      console.Text += Environment.NewLine + "1000 = " + thousand;
      var million = new mp(500000) + new mp(500000);
      console.Text += Environment.NewLine + "1 million = " + million;
      million = thousand * thousand;
      console.Text += Environment.NewLine + "1 million = " + million;
      console.Text += Environment.NewLine + "1,000,001 = " + (million + one);
      console.Text += Environment.NewLine + "1 trillion = " + million * million;
      var squareOf_65536 = (twoToThe16 * twoToThe16);
      console.Text += Environment.NewLine + "65,536^2 = " + squareOf_65536;
      var squareOf_8192 = new mp(8192) * new mp(8192);
      console.Text += Environment.NewLine + "8192^2 = " + (squareOf_8192);
      console.Text += Environment.NewLine + "7th digit of 8192^2 (from the right) = " + squareOf_8192.getDigit(6);
      var replaced = new mp(squareOf_8192);
      replaced.setDigit(6, 5);
      console.Text += Environment.NewLine + "Replace 7th digit to 5 in 8192^2 = " + replaced;
      replaced = new mp(squareOf_8192);
      replaced.setDigit(7, 5);
      console.Text += Environment.NewLine + "Replace 8th digit to 5 in 8192^2 = " + replaced;
      console.Text += Environment.NewLine + "Number of digits in " + squareOf_65536 + " = " + squareOf_65536.numDigits();
      console.Text += Environment.NewLine + "10^6 - 500,000 = " + (million - new mp(500000));
      console.Text += Environment.NewLine + "500,000 - 10^6 = " + (new mp(500000) - million);
      return true;
    }

    Boolean test_mp2()
    {
      console.Text = "";
      mp one = new mp(1);
      mp two = one + one;
      mp twoToThe16 = two.pow(16);
      var squareOf_8192 = new mp(8192) * new mp(8192);
      var squareOf_65536 = (twoToThe16 * twoToThe16);
      {
        mp divisor = new mp(2);
        for (int ii = 31; ii >= 0; --ii)
        {
          console.Text += Environment.NewLine + "2^" + ii + " = " + squareOf_65536 / divisor;
          console.Text += " with a remainder of " + squareOf_65536 % divisor;
          divisor = divisor * (new mp(2));
          if (ii <= 12) { return true; }
        }
      }
      console.Text += Environment.NewLine + "" + squareOf_8192 + " / " + new mp(11) + " = " + (squareOf_8192 / new mp(11)) + " with a remainder of " + (squareOf_8192 % new mp(11));
      return true;
    }

    Boolean test_mp3()
    {
      console.Text = "";
      mp one = new mp(1);
      mp two = one + one;
      mp twoToThe16 = two.pow(16);
      var squareOf_8192 = new mp(8192) * new mp(8192);
      var squareOf_65536 = (twoToThe16 * twoToThe16);
      {
        mp divisor = new mp(2);
        for (int ii = 31; ii >= 0; --ii)
        {
          if (ii <= 11)
          {
            console.Text += Environment.NewLine + "2^" + ii + " = " + squareOf_65536 / divisor;
            console.Text += " with a remainder of " + squareOf_65536 % divisor;
          }
          divisor = divisor * (new mp(2));
        }
      }
      console.Text += Environment.NewLine + "" + squareOf_8192 + " / " + new mp(11) + " = " + (squareOf_8192 / new mp(11)) + " with a remainder of " + (squareOf_8192 % new mp(11));
      return true;
    }

    private void ResetCalcPanel(bool resetState = true)
    {
      enterIntegerLbl.Text = "Enter a whole number: ";
      okBtn.Text = "OK";
      cancelBtn.Visible = false;
      ShowCalcPanel(false);
      EnableCalcPanel(true);
      if (resetState) { calc = new CalculatorState(); }
      undoBtn.Visible = (calc.numberStack.Count > 0);
      redoBtn.Visible = (calc.redoStack.Count > 0);
    }

    private bool test_rational()
    {
      console.Text = "Testing rational numbers:";
      Rational zero = new Rational();
      console.Text += Environment.NewLine + Environment.NewLine + "Zero = " + zero;
      Rational one = new Rational();
      one = one + new Rational(1, 2);
      one = one + new Rational(1, 3);
      one = one + new Rational(1, 6);
      console.Text += Environment.NewLine + "One = " + one;
      console.Text += Environment.NewLine + "Prime factorization of one = " + one.printFactors();
      console.Text += Environment.NewLine + "Prime factorization of -1 = " + (-one).printFactors();
      Rational half = new Rational(-1, 4) * new Rational(4, -2);
      console.Text += Environment.NewLine + "One half = " + half;
      Rational thePower = new Rational(half);
      for (int i = 0; i < 6; ++i)
      {
        var newPower = thePower * thePower;
        console.Text += Environment.NewLine + "" + thePower + "^2 = " + newPower;
        thePower = newPower;
      }
      Rational twelve = new Rational(36, 3);
      console.Text += Environment.NewLine + "Prime factorization of twelve = " + twelve.printFactors();
      Rational minusTwelve = new Rational(24, -2);
      console.Text += Environment.NewLine + "Prime factorization of negative twelve = " + minusTwelve.printFactors();
      Rational oneOver2048 = new Rational(2, 4096);
      console.Text += Environment.NewLine + "Prime factorization of 1 / 2048 = " + oneOver2048.printFactors();
      Rational hundred = new Rational(1000, 10);
      console.Text += Environment.NewLine + "Prime factorization of 100 = " + hundred.printFactors();
      Rational myNum = new Rational(-24, 138);
      console.Text += Environment.NewLine + "Prime factorization of -24 / 138 = " + myNum.printFactors();
      return true;
    }

    bool test_quadratic()
    {
      console.Text = "Testing sums of square roots:";
      var zero = new QuadraticNumber();
      console.Text += Environment.NewLine + Environment.NewLine + "Zero = " + zero;
      zero = QuadraticNumber.sqrt(9) - new QuadraticNumber(new Rational(3));
      console.Text += Environment.NewLine + "Zero = " + zero;
      var twoThirds = new QuadraticNumber(new Rational(2, 3));
      console.Text += Environment.NewLine + "Two-thirds = " + twoThirds;
      var one = QuadraticNumber.sqrt(1);
      console.Text += Environment.NewLine + "Square root of 1 = " + one;
      var sqrt2 = QuadraticNumber.sqrt(2);
      console.Text += Environment.NewLine + "Square root of 2 = " + sqrt2;
      var sqrt36 = QuadraticNumber.sqrt(36);
      console.Text += Environment.NewLine + "Square root of 36 = " + sqrt36;
      var sqrt12 = QuadraticNumber.sqrt(12);
      console.Text += Environment.NewLine + "Square root of 12 = " + sqrt12;
      console.Text += Environment.NewLine + "Twelve is " + (sqrt12 * sqrt12);
      Rational rationalOut = new Rational();
      bool twoThirdsIsRational = twoThirds.getRational(ref rationalOut);
      if (!twoThirdsIsRational) { return false; }
      console.Text += Environment.NewLine + "Square root of 2/3 = " + QuadraticNumber.sqrt(rationalOut);
      var goldenRatio = QuadraticNumber.sqrt(new Rational(5, 4)) + new QuadraticNumber(new Rational(1, 2));
      console.Text += Environment.NewLine + "The golden ratio is " + goldenRatio;
      var oneOverGolden = QuadraticNumber.sqrt(new Rational(5, 4)) - new QuadraticNumber(new Rational(1, 2));
      console.Text += Environment.NewLine + "One = " + (goldenRatio * oneOverGolden);
      oneOverGolden = new QuadraticNumber(new Rational(1)) / goldenRatio;
      console.Text += Environment.NewLine + "The reciprocal golden ratio is:";
      console.Text += Environment.NewLine + "" + oneOverGolden;
      var sumOfSquareRoots = QuadraticNumber.sqrt(2) + QuadraticNumber.sqrt(3) + new QuadraticNumber(new Rational(1));
      var reciprocal = new QuadraticNumber(new Rational(1)) / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);
      return true;
    }

    bool test_quadratic2()
    {
      var sumOfSquareRoots = QuadraticNumber.sqrt(5) - QuadraticNumber.sqrt(3) + new QuadraticNumber(new Rational(1));
      var reciprocal = new QuadraticNumber(new Rational(1)) / sumOfSquareRoots;
      console.Text = "The reciprocal of " +
        sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);
      sumOfSquareRoots = QuadraticNumber.sqrt(7) + QuadraticNumber.sqrt(3) + new QuadraticNumber(new Rational(1));
      reciprocal = new QuadraticNumber(new Rational(1)) / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);
      var unity = new QuadraticNumber(new Rational(1));
      sumOfSquareRoots = QuadraticNumber.sqrt(7) + QuadraticNumber.sqrt(5) + QuadraticNumber.sqrt(3) + unity;
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);

      sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 5; ++ii)
      {
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
      }
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);
      sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 6; ++ii)
      {
        QuadraticNumber coeff = new QuadraticNumber(new Rational(-1));
        if ((ii % 2) == 0) { coeff = coeff * coeff; }
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii) * coeff;
      }
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);

      sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 5; ++ii)
      {
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
      }
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      var recipIntegral = reciprocal.factorAsIntegral();
      console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
      console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
        new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));
      return true;
    }

    bool test_quadratic3()
    {
      console.Text = "";
      var unity = new QuadraticNumber(new Rational(1));
      var sumOfSquareRoots = new QuadraticNumber();
      QuadraticNumber minusOne = new QuadraticNumber(new Rational(-1));
      for (int ii = 0; ii < 6; ++ii)
      {
        QuadraticNumber coeff = new QuadraticNumber(minusOne);
        if ((ii % 2) == 0) { coeff = coeff * coeff; }
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii) * coeff;
      }
      var reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      var recipIntegral = reciprocal.factorAsIntegral();
      console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
      console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
        new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));

      sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 6; ++ii)
      {
        QuadraticNumber coeff = new QuadraticNumber(minusOne);
        if ((ii % 2) == 1) { coeff = coeff * coeff; }
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii) * coeff;
      }
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      recipIntegral = reciprocal.factorAsIntegral();
      console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
      console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
        new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));

      for (int lim_ = 7; lim_ < 8; ++lim_)
      {
        sumOfSquareRoots = new QuadraticNumber();
        for (int ii = 0; ii < lim_; ++ii)
        {
          sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
        }
        reciprocal = unity / sumOfSquareRoots;
        console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
        console.Text += Environment.NewLine + "" + reciprocal;
        recipIntegral = reciprocal.factorAsIntegral();
        console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
        console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
          new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));
      }
      return true;
    }

    bool test_quadratic4()
    {
      console.Text = "";
      var unity = new QuadraticNumber(new Rational(1));
      QuadraticNumber minusOne = new QuadraticNumber(new Rational(-1));
      var sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 6; ++ii)
      {
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
      }
      var reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      console.Text += Environment.NewLine + "One = " + (reciprocal * sumOfSquareRoots);
      sumOfSquareRoots = new QuadraticNumber();
      for (int ii = 0; ii < 6; ++ii)
      {
        sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
      }
      reciprocal = unity / sumOfSquareRoots;
      console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
      console.Text += Environment.NewLine + "" + reciprocal;
      var recipIntegral = reciprocal.factorAsIntegral();
      console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
      console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
        new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));
      console.Text += Environment.NewLine + "The next test lasts a long time. \"End Current Test\" to skip it.";
      return true;
    }

    bool test_quadratic5()
    {
      var unity = new QuadraticNumber(new Rational(1));
      console.Text = "";
      for (int lim_ = 8; lim_ < 11; ++lim_)
      {
        var sumOfSquareRoots = new QuadraticNumber();
        for (int ii = 0; ii < lim_; ++ii)
        {
          sumOfSquareRoots = sumOfSquareRoots + QuadraticNumber.sqrt(ii);
        }
        var reciprocal = unity / sumOfSquareRoots;
        console.Text += Environment.NewLine + "The reciprocal of " + sumOfSquareRoots + " is:";
        console.Text += Environment.NewLine + "" + reciprocal;
        var recipIntegral = reciprocal.factorAsIntegral();
        console.Text += Environment.NewLine + "This equals " + recipIntegral.Key.ToString(true) + " / " + recipIntegral.Value;
        console.Text += Environment.NewLine + "One = " + (sumOfSquareRoots * recipIntegral.Key *
          new QuadraticNumber(new Rational(new mp(1), recipIntegral.Value)));
      }
      return true;
    }

    private void endTests()
    {
      console.Text = "";
      testState = new TestingState();
      testBtn.Visible = true;
      continueBtn.Visible = false;
      endCurrentTest.Visible = false;
      numberInput.Visible = true;
      enterIntegerLbl.Visible = true;
      okBtn.Visible = true;
      ResetCalcPanel();
    }

    private int totalNumTests = 3;

    private bool incrementTest()
    {
      if (testState.whichTest < 0) { return false; }
      if (testState.whichTest >= totalNumTests - 1)
      {
        return true;
      }
      testState.testState = 0;
      testState.whichTest++;
      return runTests();
    }

    private bool runTests()
    {
      testBtn.Visible = false;
      continueBtn.Visible = true;
      endCurrentTest.Visible = true;
      numberInput.Visible = false;
      enterIntegerLbl.Visible = false;
      okBtn.Visible = false;
      ResetCalcPanel();

      if (testState.whichTest == 0)
      {
        if (testState.testState == 0) { test_mp(); }
        else if (testState.testState == 1) { test_mp2(); }
        else if (testState.testState == 2) { test_mp3(); }
        else if (testState.testState > 0)
        {
          incrementTest();
          return true;
        }
      }
      else if (testState.whichTest == 1)
      {
        if (testState.testState == 0) { test_rational(); }
        else if (testState.testState > 0)
        {
          incrementTest();
          return true;
        }
      }
      else if (testState.whichTest == totalNumTests - 1) // 2
      {
        if (testState.testState == 0) { test_quadratic(); }
        else if (testState.testState == 1) { test_quadratic2(); }
        else if (testState.testState == 2) { test_quadratic3(); }
        else if (testState.testState == 3) { test_quadratic4(); }
        else if (testState.testState == 4) { test_quadratic5(); }
        else if (testState.testState == 5)
        {
          console.Text = "Done.";
        }
        else if (testState.testState > 0)
        {
          endTests();
          return false;
        }
      }
      else if (testState.whichTest < 0) { endTests(); return false; }
      return false;
    }

    public MainWindow()
    {
      InitializeComponent();
      endTests();
    }
  }
}
