namespace function_calculator_cs
{
  public partial class MainWindow : Form
  {

    Boolean test_mp()
    {
      mp zero = mp.zero();
      console.Text += Environment.NewLine + "Testing multiprecision integers:";
      console.Text += Environment.NewLine + "0 = " + zero;
      mp one = new mp(1);
      mp two = one + one;
      console.Text += Environment.NewLine + "1 = " + one + Environment.NewLine + "2 = " + two;
      mp twoToThe16 = two.pow(16);
      console.Text += Environment.NewLine + "65,536 = " + twoToThe16;
      mp thousand = new mp(1000);
      /*console.Text += Environment.NewLine + "1000 = " << thousand;
      auto million = mp(500000) + mp(500000);
      console.Text += Environment.NewLine + "1 million = " << million;
      million = thousand * thousand;
      console.Text += Environment.NewLine + "1 million = " << million;
      console.Text += Environment.NewLine + "1,000,001 = " << (million + one);
      console.Text += Environment.NewLine + "1 trillion = " << million * million;
      auto squareOf_65536 = (twoToThe16 * twoToThe16);
      console.Text += Environment.NewLine + "65,536^2 = " << squareOf_65536;
      auto squareOf_8192 = mp(8192) * mp(8192);
      console.Text += Environment.NewLine + "8192^2 = " << (squareOf_8192);
      console.Text += Environment.NewLine + "6th digit of 8192^2 = " << squareOf_8192.getDigit(6);
      auto replaced = squareOf_8192;
      replaced.setDigit(6, 5);
      console.Text += Environment.NewLine + "Replace 6th digit to 5 in 8192^2 = " << replaced;
      replaced = squareOf_8192;
      replaced.setDigit(7, 5);
      console.Text += Environment.NewLine + "Replace 7th digit to 5 in 8192^2 = " << replaced;
      console.Text += Environment.NewLine + "Number of digits in " << squareOf_65536 << " = " << squareOf_65536.numDigits();
      console.Text += Environment.NewLine + "10^6 - 500,000 = " << million - mp(500000);
      console.Text += Environment.NewLine + "500,000 - 10^6 = " << mp(500000) - million;
      {
        mp divisor(2);
        for (int ii = 31; ii >= 0; --ii)
        {
          console.Text += Environment.NewLine + "2^" << ii << " = " << squareOf_65536 / divisor;
          console.Text += " with a remainder of " << squareOf_65536 % divisor;
          divisor = divisor * mp(2);
        }
      }
      console.Text += Environment.NewLine + "" << squareOf_8192 << " / " << mp(11) << " = " << (squareOf_8192 / mp(11)) << " with a remainder of " << (squareOf_8192 % mp(11));
      */
      return true;
    }

    public MainWindow()
    {
      InitializeComponent();
      test_mp();
    }
  }
}
