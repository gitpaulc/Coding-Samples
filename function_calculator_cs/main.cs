namespace function_calculator_cs
{
  internal static class FunctionMain
  {
    [STAThread]
    static void Main()
    {
      ApplicationConfiguration.Initialize();
      Application.Run(new MainWindow());
    }
  }
}