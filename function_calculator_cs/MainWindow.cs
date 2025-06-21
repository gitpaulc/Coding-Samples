namespace function_calculator_cs
{
  public partial class MainWindow : Form
  {

    Boolean test_mp()
    {
      mp zero = mp.zero();
      console.Text += "\n0 = " + zero.ToString();
      /*mp one = 1;
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
      std::cout << "\n6th digit of 8192^2 = " << squareOf_8192.getDigit(6);
      auto replaced = squareOf_8192;
      replaced.setDigit(6, 5);
      std::cout << "\nReplace 6th digit to 5 in 8192^2 = " << replaced;
      replaced = squareOf_8192;
      replaced.setDigit(7, 5);
      std::cout << "\nReplace 7th digit to 5 in 8192^2 = " << replaced;
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
