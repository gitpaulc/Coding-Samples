
namespace PlayParser
{
  public static class Program
  {
    private static PlayParserForm? ppf = null;

    public static bool IsUppercase(string str)
    {
      if (str.Length == 0) { return false; }
      var firstLet = str.Substring(0, 1);
      if (firstLet.ToLower() == firstLet) { return false; }
      return true;
    }

    public static string NumStringToWordString(string numString)
    {
      if (numString == "1") { numString = "One"; }
      if (numString == "2") { numString = "Two"; }
      if (numString == "3") { numString = "Three"; }
      if (numString == "4") { numString = "Four"; }
      if (numString == "5") { numString = "Five"; }
      if (numString == "6") { numString = "Six"; }
      if (numString == "7") { numString = "Seven"; }
      if (numString == "8") { numString = "Eight"; }
      if (numString == "9") { numString = "Nine"; }
      if (numString == "10") { numString = "Ten"; }
      if (numString == "11") { numString = "Eleven"; }
      if (numString == "12") { numString = "Twelve"; }
      return numString;
    }

    public static bool IsRomanNumeral(string str)
    {
      if (str == "IV") { return true; }
      if (str == "VI") { return true; }
      if (str == "VII") { return true; }
      if (str == "VIII") { return true; }
      if (str == "IX") { return true; }
      if (str == "XI") { return true; }
      if (str == "XII") { return true; }
      if (str == "XIII") { return true; }
      return false;
    }

    public static string ToCamelCase(string str)
    {
      string strOut = "";
      var words = str.Split(' ');
      int ii = -1;
      foreach (var word in words)
      {
        if (word.Length == 0) { continue; }
        ++ii;
        if (ii > 0) { strOut += " "; }
        if (word.Length < 2) { strOut += word.ToUpper(); continue; }
        if (word == "of") { strOut += word; continue; }
        if (IsRomanNumeral(word)) { strOut += word.ToUpper(); continue; }
        var str0 = word.Substring(0, 1).ToUpper();
        str0 += word.Substring(1).ToLower();
        strOut += str0;
      }
      return strOut;
    }

    public static List<string> ReadFileAsLines(string filePath)
    {
      var lines = new List<string>();
      try
      {
        using (StreamReader sr = new StreamReader(filePath))
        {
          string? line = "";
          while ((line = sr.ReadLine()) != null)
          {
            lines.Add(line);
          }
        }
      }
      catch (System.Exception)
      {

      }
      return lines;
    }

    public enum PrintMode
    {
      ConsoleOnly = 0,
      FileOnly = 1,
      ConsoleAndFile = 2
    }
    public static void Print(List<string> strs, PrintMode pm = PrintMode.ConsoleOnly,
      string fileName = "", bool append = false)
    {
      bool toConsole = (pm == PrintMode.ConsoleOnly) || (pm == PrintMode.ConsoleAndFile);
      bool toFile = (pm == PrintMode.FileOnly) || (pm == PrintMode.ConsoleAndFile);
      if (toConsole)
      {
        foreach (string str in strs)
        {
          Console.WriteLine(str);
        }
      }
      if (toFile && (fileName.Length > 0))
      {
        using (StreamWriter sw = new StreamWriter(fileName, append))
        {
          foreach (string str in strs)
          {
            sw.WriteLine(str);
          }
        }
      }
    }

    public static void Print(string str, PrintMode pm = PrintMode.ConsoleOnly,
      string fileName = "", bool append = false)
    {
      var strs = new List<string>();
      strs.Add(str);
      Print(strs, pm, fileName, append);
    }


    public static string GetPlaysFolder()
    {
      var baseDir = AppDomain.CurrentDomain.BaseDirectory;
      return Path.GetFullPath(Path.Combine(baseDir, @"..\..\..\..\PlayParser\Plays"));
    }

    public static List<Play> RunPlays()
    {
      var result = new List<Play>();
      var playNames = new SortedSet<string>();
      for (int ii = 0; ii < (int)Play.PlayEnum.NumPlays; ++ii)
        playNames.Add(Play.GetPlayName((Play.PlayEnum)ii));

      var rootFolder = GetPlaysFolder();
      foreach (var playName in playNames)
      {
        Play play = new Play();
        play.ResetPlay(playName);
        var playFolder = rootFolder + "\\" + playName;
        play.CopyTrimmedScenes(playFolder);
        play.WriteCounts(playFolder);
        play.DetectGenders(playFolder);
        play.PrintStatistics(playFolder);
        result.Add(play);
      }
      return result;
    }

    public static PlayParserForm? GetPlayParserForm() { return ppf; }

    [STAThread]
    public static void Main()
    {
      Application.EnableVisualStyles();
      Application.SetCompatibleTextRenderingDefault(false);
      using (var splash = new SplashForm())
        splash.ShowDialog();
      ppf = new PlayParserForm();
      Application.Run(ppf);
    }
  }
}
