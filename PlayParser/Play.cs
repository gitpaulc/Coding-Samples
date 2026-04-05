

namespace PlayParser
{
  public class Play
  {
    public enum PlayEnum
    {
      Hamlet = 0,
      Henry1 = 1,
      Henry2 = 2,
      NumPlays = 3
    }

    public static string GetPlayName(PlayEnum plEnum)
    {
      if (plEnum == PlayEnum.Henry1) { return "King Henry IV Part 1"; }
      if (plEnum == PlayEnum.Henry2) { return "King Henry IV Part 2"; }
      //if (plEnum == PlayEnum.Hamlet)
      return "Hamlet";
    }

    public string playName = "";
    public SortedSet<string> Actors = new SortedSet<string>();
    public Dictionary<string, List<string> > scenesPresent = new Dictionary<string, List<string>>();
    public Dictionary<string, Dictionary<string, int> > lineCounts = new Dictionary<string, Dictionary<string, int> >();

    public void ResetPlay(string nameOfPlay)
    {
      playName = nameOfPlay;
      Actors.Clear();
      scenesPresent.Clear();
      lineCounts.Clear();
    }

    public Dictionary<string, List<string> > GetActorsInEachScene()
    {
      var sceneActors = new Dictionary<string, List<string>>();
      foreach (var kvp in scenesPresent)
      {
        foreach (var scene in kvp.Value)
        {
          if (!sceneActors.ContainsKey(scene))
          {
            sceneActors[scene] = new List<string>();
          }
          sceneActors[scene].Add(kvp.Key);
          sceneActors[scene].Sort();
        }
      }
      return sceneActors;
    }

    public Dictionary<string, int> TotalLineCounts()
    {
      var totals = new Dictionary<string, int>();
      foreach (var kvp in lineCounts)
      {
        totals[kvp.Key] = 0;
        foreach (var kvp_ in kvp.Value)
        {
          totals[kvp.Key] += kvp_.Value;
        }
      }
      return totals;
    }

    bool DetectEnterInGetActors(string line)
    {
      var words = line.Split(' ');
      string enterWord = "Enter";
      foreach (var word in words)
      {
        if (word.Equals(enterWord))
        {
          return true;
        }
      }
      return false;
    }

    bool ActorMatchesLine(string actor, string line)
    {
      var ac = Program.ToCamelCase(actor);
      var ln = Program.ToCamelCase(line);
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        if (ln == "Cornelius Voltimand") { return (ac.Equals("Cornelius") || ac.Equals("Voltimand")); }
        else if (ln == "Lord Polonius") { ln = "Polonius"; }
        else if (ln == "Prince Fortinbras") { ln = "Fortinbras"; }
      }
      else if (playName == GetPlayName(PlayEnum.Henry1))
      {
        if (ln == "Sir Walter Blunt") { ln = "Blunt"; }
      }
      return ac.Equals(ln);
    }

    void RefineActorsCalculation(string folderOut)
    {
      SortedSet<string> OldActors = new SortedSet<string>();
      foreach (var actor in Actors)
      {
        OldActors.Add(actor);
      }
      Actors.Clear();
      foreach (var actor in OldActors)
      {
        if (actor.Length == 0) { continue; }
        Actors.Add(actor);
        if (actor[actor.Length - 1] != 's') { continue; }
        var singular = actor.Substring(0, actor.Length - 1);

        List<int> counts = new List<int>();
        for (int ii = 0; ii < 5; ++ii)
        {
          counts.Add(0);
        }
        Program.Print("Refining " + actor + " occurrences.");

        var paths = System.IO.Directory.GetFiles(folderOut);
        bool found = false;
        bool foundFull = false;
        foreach (var path in paths)
        {
          var filename = System.IO.Path.GetFileName(path).Trim();
          var lines = Program.ReadFileAsLines(path);
          foreach (var line in lines)
          {
            if (!foundFull)
            {
              if (ActorMatchesLine(actor, line)) {  foundFull = true; }
            }
            string prefix = "First ";
            for (int ii = 0; ii < 5; ++ii)
            {
              if (ii + 1 == 2) { prefix = "Second "; }
              else if (ii + 1 == 3) { prefix = "Third "; }
              else if (ii + 1 == 4) { prefix = "Fourth "; }
              else if (ii + 1 == 5) { prefix = "Fifth "; }
              if (ActorMatchesLine(prefix + singular, line))
              {
                counts[ii]++;
                Actors.Add(prefix + singular);
                if (!found)
                {
                  Actors.Remove(actor);
                  found = true;
                }
              }
            }
          }
        }
        if (!foundFull)
        {
          Actors.Remove(actor);
        }
      }
    }

    string ReplaceLineInGetActors(string playName, string line)
    {
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        line = line.Replace("captain", "Captain");
        line = line.Replace("the English Ambassadors", "Ambassadors");
      }
      else if (playName == GetPlayName(PlayEnum.Henry1))
      {
        if (line.Contains("Alarum"))
        {
          if (line.Contains("to the battle"))
          {
            line = "KING HENRY, EARL OF DOUGLAS, and SIR WALTER BLUNT";
          }
          else if (line.Contains("solus"))
          {
            line = "FALSTAFF";
          }
          else if (line.Contains("xcursions"))
          {
            line = line.Replace("Alarum. Excursions. ", "");
          }
        }
      }
      else if (playName == GetPlayName(PlayEnum.Henry2))
      {
        if (line.Contains("Alarum"))
        {
          if (line.Contains("xcursions"))
          {
            line = line.Replace("Alarum. Excursions. ", "");
          }
        }
      }
      return line;
    }

    bool ShouldReturnByLineInGetActors(string playName, string line)
    {
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        if (line.Contains("very lovingly")) { return true; }
      }
      return false;
    }

    string ReplaceWordInGetActors(string playName, string word)
    {
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        if (word == "Attendants With Foils") { word = "Attendants"; }
      }
      else if (playName == GetPlayName(PlayEnum.Henry1))
      {
        if (word == "King Henry") { word = "King Henry IV"; }
      }
      return word;
    }

    private enum WordRes
    {
      Okay = 0,
      Continue = 1,
      Return = 2
    }

    WordRes ShouldReturnByWordInGetActors(string playName, string word)
    {
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        if (word == "Queen Margaret") { return WordRes.Continue; }
        if (word == "Anon") { return WordRes.Return; } // 3.2
      }
      return WordRes.Okay;
    }

    string SeekMatch(string playName, string word)
    {
      List<string> matches = new List<string>();
      if (playName == GetPlayName(PlayEnum.Hamlet))
      {
        matches.Add("King Claudius");
        matches.Add("Francisco");
      }
      else if (playName == GetPlayName(PlayEnum.Henry1))
      {
        matches.Add("King Henry IV");
        matches.Add("Sir Walter Blunt");
        matches.Add("Douglas");
        matches.Add("Falstaff");
        matches.Add("Lancaster");
        matches.Add("Westmoreland");
        matches.Add("Vernon");
        matches.Add("Worcester");
        foreach (var match in matches)
        {
          if (match.Contains(word)) { return match; }
        }
      }
      foreach (var match in matches)
      {
        if (word.Contains(match)) { return match; }
      }
      return word;
    }

    void GetActors(string line)
    {
      if (!DetectEnterInGetActors(line)) { return; }
      line = line.Replace("Enter ", "");
      line = line.Replace(" two ", " ");
      line = line.Replace("two ", "");
      line = ReplaceLineInGetActors(playName, line);
      if (ShouldReturnByLineInGetActors(playName, line)) { return; }
      line = line.Replace(" a ", " ");
      line = line.Replace(", a ", ", ");
      line = line.Replace(" and ", ", ");
      line = line.Replace(", and", ", ");
      var words = line.Split(", ");
      foreach (var word0 in words)
      {
        if (word0.Length == 0) { continue; }
        var word = word0;
        if (word0[word0.Length - 1] == ',')
        {
          word = word0.Substring(0, word0.Length - 1);
        }
        if (word.Length == 0) { continue; }
        if (word.Equals("Enter")) { continue; }
        if (!Program.IsUppercase(word)) { continue; }
        word = Program.ToCamelCase(word);
        if (word == "A") { continue; }
        if (word == "The") { continue; }
        if (word == "He") { continue; }
        if (word == "She") { continue; }
        word = SeekMatch(playName, word);
        word = ReplaceWordInGetActors(playName, word);
        {
          var wordRes = ShouldReturnByWordInGetActors(playName, word);
          if (wordRes == WordRes.Continue) { continue; }
          if (wordRes == WordRes.Return) { return; }
        }
        Actors.Add(word);
      }
    }

    List<string> PreprocessLines(List<string> lines)
    {
      var linesOut = new List<string>();
      foreach (var line in lines)
      {
        if (playName == GetPlayName(PlayEnum.Hamlet))
        {
          if (line.Contains("Enter two Players, King and Queen"))
          {
            linesOut.Add("Enter Player King, Player Queen");
            continue;
          }
          if (line.Contains("the Corpse of OPHELIA"))
          {
            linesOut.Add(line.Replace("the Corpse of OPHELIA", "Ophelia"));
            continue;
          }
          if (line.Contains("First Priest"))
          {
            linesOut.Add("Priest");
            continue;
          }
        }
        else if (playName == GetPlayName(PlayEnum.Henry1))
        {
          if (line.Contains("DOUGLAS kills SIR WALTER BLUNT"))
          {
            linesOut.Add("Exit SIR WALTER BLUNT");
            linesOut.Add("Enter HOTSPUR");
            continue;
          }
        }
        else if (playName == GetPlayName(PlayEnum.Henry2))
        {
          if (line.Contains("KING HENRY V"))
          {
            linesOut.Add(line.Replace("KING HENRY V", "PRINCE HENRY"));
            continue;
          }
        }
        linesOut.Add(line);
      }
      return linesOut;
    }

    public void CopyTrimmedScenes(string folderName)
    {
      var folderOut = folderName + "\\ScenesOut";
      if (!(System.IO.Directory.Exists(folderOut)))
      {
        System.IO.Directory.CreateDirectory(folderOut);
      }
      var paths = System.IO.Directory.GetFiles(folderName + "\\ScenesIn");
      foreach (var path in paths)
      {
        var filename = System.IO.Path.GetFileName(path).Trim();
        Program.Print("Copying " + filename + ".");
        var lines = Program.ReadFileAsLines(path);
        lines = PreprocessLines(lines);
        foreach (var line in lines)
        {
          GetActors(line);
        }
        Program.Print(lines, Program.PrintMode.FileOnly, folderOut + "\\" + filename, false);
      }

      RefineActorsCalculation(folderOut);

      var actorsFilename = folderName + "\\Actors.txt";
      Program.Print("Writing " + actorsFilename + ".");
      Program.Print("Dramatis Personae.", Program.PrintMode.FileOnly, actorsFilename, false);
      Program.Print(Actors.ToList<string>(), Program.PrintMode.FileOnly, actorsFilename, true);
    }

    public bool NotSpeaking(string actor, string line)
    {
      var ln = Program.ToCamelCase(line);
      if (ln.Contains("Exeunt")) { return true; }
      if (ln.Contains("Exit")) { return true; }
      foreach (var actor2 in Actors)
      {
        if (actor == actor2) { continue; }
        if (ActorMatchesLine(actor2, line))
        {
          return true;
        }
      }
      return false;
    }

    bool LineShouldBeAdded(string line)
    {
      if (line.Length == 0) { return false; }
      if (line.StartsWith("Enter ")) { return false; }
      if (line.StartsWith("Exit ")) { return false; }
      if (line.StartsWith("Exeunt")) { return false; }
      return true;
    }

    public void WriteCounts(string folderName)
    {
      var folderOut = folderName + "\\ScenesOut";
      var paths = System.IO.Directory.GetFiles(folderOut);
      foreach (var actor in Actors)
      {
        scenesPresent[actor] = new List<string>();
        lineCounts[actor] = new Dictionary<string, int>();

        foreach (var path in paths)
        {
          int lineCount = 0;
          bool inScene = false;
          bool theSceneWasStarted = false;
          bool speaking = false;
          var filename = System.IO.Path.GetFileName(path).Trim();

          var lines = Program.ReadFileAsLines(path);
          foreach (var line in lines)
          {
            if (theSceneWasStarted && (line.StartsWith("Shakespeare homepage"))) { break; }
            if (speaking)
            {
              if (NotSpeaking(actor, line)) { speaking = false; }
            }
            if (speaking)
            {
              if (LineShouldBeAdded(line))
              {
                lineCount++;
              }
            }
            if (ActorMatchesLine(actor, line))
            {
              inScene = true;
              speaking = true;
              theSceneWasStarted = true;
            }
            if (line.Contains("Enter "))
            {
              if (Program.ToCamelCase(line).Contains(actor))
              {
                inScene = true;
              }
            }
          }
          if (inScene) { scenesPresent[actor].Add(filename); }
          if (lineCount > 0) { lineCounts[actor][filename] = lineCount; }
        }
      }
    }

    public class Descending : IComparer<int>
    {
      public int Compare(int x, int y) { return y.CompareTo(x); }
    }

    SortedDictionary<int, List<string> > TotalCountsSorted()
    {
      var sortedCounts = new SortedDictionary<int, List<string> >(new Descending());
      var totalCounts = TotalLineCounts();
      foreach (var kvp in totalCounts)
      {
        if (!sortedCounts.ContainsKey(kvp.Value))
        {
          sortedCounts[kvp.Value] = new List<string>();
        }
        sortedCounts[kvp.Value].Add(kvp.Key);
      }
      return sortedCounts;
    }

    public void PrintStatistics(string folderName)
    {
      List<string> lines = new List<string>();
      lines.Add("Dramatis Personae (Alphabetical).");
      lines.Add("");
      foreach (var actor in Actors)
      {
        lines.Add(actor);
      }
      lines.Add("");
      lines.Add("Dramatis Personae (by Number of Lines).");
      lines.Add("");
      var totalCounts = TotalCountsSorted();
      foreach (var kvp in totalCounts)
      {
        foreach (var actor in kvp.Value)
        {
          lines.Add(actor + $" ({kvp.Key} lines)");
        }
      }
      lines.Add("");
      lines.Add("Number of Lines by Scene.");
      foreach (var actor in Actors)
      {
        lines.Add("");
        lines.Add(actor + ".");
        foreach (var scene in scenesPresent[actor])
        {
          var actNum = scene.Substring(0, 1);
          var sceneNum = scene.Substring(2, 1);
          actNum = Program.NumStringToWordString(actNum);
          sceneNum = Program.NumStringToWordString(sceneNum);
          var sceneString = "Act " + actNum + ", Scene " + sceneNum + ".";
          if (!lineCounts[actor].ContainsKey(scene))
          {
            lines.Add(sceneString + " No lines, entrance only.");
            continue;
          }
          var lineCount = lineCounts[actor][scene];
          lines.Add(sceneString + " " + lineCount + " lines.");
        }
      }

      lines.Add("");
      lines.Add("Dramatis Personae by Scene.");
      var actorsInEachScene = GetActorsInEachScene();
      var paths = System.IO.Directory.GetFiles(folderName + "\\ScenesIn");
      foreach (var path in paths)
      {
        var filename = System.IO.Path.GetFileName(path).Trim();
        var actorsHere = actorsInEachScene[filename];
        var actNum = filename.Substring(0, 1);
        var sceneNum = filename.Substring(2, 1);
        actNum = Program.NumStringToWordString(actNum);
        sceneNum = Program.NumStringToWordString(sceneNum);
        var sceneString = "Act " + actNum + ", Scene " + sceneNum + ": ";
        int ii = -1;
        foreach (var actor in actorsHere)
        {
          ++ii;
          if (ii > 0)
          {
            if (actorsHere.Count >= 3) { sceneString += ","; }
            sceneString += " ";
          }
          if (ii == actorsHere.Count - 1)
          {
            sceneString += "and ";
          }
          sceneString += actor;
        }
        sceneString += ".";
        lines.Add(sceneString);
      }

      Program.Print(lines, Program.PrintMode.FileOnly, folderName + "\\" + playName + "Lines.txt", false);
    }

  }
}
