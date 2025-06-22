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
      okBtn.Text = "OK";
      cancelBtn.Visible = false;
      ShowCalcPanel(false);
      EnableCalcPanel(true);
      if (resetState) { calc = new CalculatorState(); }
      undoBtn.Visible = (calc.numberStack.Count > 0);
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

    private void runTests()
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
        else if (testState.testState == 3)
        {
          console.Text = "";
          console.Text = Environment.NewLine + "Done.";
        }
        else if (testState.testState > 0)
        {
          endTests(); return;
        }
      }
      else if (testState.whichTest < 0) { endTests(); return; }
    }

    public MainWindow()
    {
      InitializeComponent();
      endTests();
    }
  }
}
