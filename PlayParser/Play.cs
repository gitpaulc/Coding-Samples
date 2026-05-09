

namespace PlayParser
{
  public class Play
  {
    // Returns play names by scanning the Plays folder — no hardcoded list.
    public static List<string> GetAllPlayNames()
    {
      var folder = Program.GetPlaysFolder();
      if (!System.IO.Directory.Exists(folder)) return [];
      return System.IO.Directory.GetDirectories(folder)
        .Select(System.IO.Path.GetFileName)
        .Where(n => n != null)
        .Select(n => n!)
        .OrderBy(n => n)
        .ToList();
    }

    public string playName = "";
    public string author   = "";
    public SortedSet<string> Actors = new SortedSet<string>();
    public Dictionary<string, List<string> > scenesPresent = new Dictionary<string, List<string>>();
    public Dictionary<string, Dictionary<string, int> > lineCounts = new Dictionary<string, Dictionary<string, int> >();
    public Dictionary<string, ActorGender> actorGenders = new Dictionary<string, ActorGender>();

    public void ResetPlay(string nameOfPlay)
    {
      playName = nameOfPlay;
      author   = "";
      Actors.Clear();
      scenesPresent.Clear();
      lineCounts.Clear();
      actorGenders.Clear();
    }

    public void LoadAuthor(string folderName)
    {
      var lines = Program.ReadFileAsLines(folderName + "\\Author.txt");
      if (lines.Count > 0) author = lines[0].Trim();
    }

    private static readonly HashSet<string> ClassicalPlays = new(StringComparer.OrdinalIgnoreCase)
    {
      "The Trojan Women"
    };

    public static string GetPlayEra(string playName)
    {
      if (ClassicalPlays.Contains(playName)) return "Classical";
      return "Renaissance";
    }

    // Returns the main character for a scene given its per-actor line counts.
    // Priority: actor whose name appears in the play title; fallback: most lines.
    public string GetSceneMainCharacter(Dictionary<string, int> sceneLineCounts)
    {
      string? titleChar = sceneLineCounts.Keys
        .FirstOrDefault(a => playName.IndexOf(a, StringComparison.OrdinalIgnoreCase) >= 0);
      if (titleChar != null) return titleChar;

      return sceneLineCounts
        .OrderByDescending(kv => kv.Value)
        .Select(kv => kv.Key)
        .FirstOrDefault() ?? "";
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

    // Speakers that represent groups rather than individual characters.
    private static readonly HashSet<string> GroupSpeakers = new(StringComparer.OrdinalIgnoreCase)
    {
      "All", "Both", "Danes", "Thieves", "Players", "Lords", "Ladies",
      "Citizens", "Soldiers", "Officers", "Servants", "Attendants",
      "Sailors", "Messengers", "Ambassadors", "Voices", "Others", "Princes"
    };

    // A speaker-label line consists entirely of ALL-CAPS words (plus "and"/"&" connectors).
    private static bool IsSpeakerLabel(string stripped)
    {
      bool hasMultiLetter = false;
      foreach (var word in stripped.Split(' '))
      {
        if (word.Length == 0) continue;
        if (word.Equals("and", StringComparison.OrdinalIgnoreCase)) continue;
        if (word == "&") continue;
        if (!word.All(c => char.IsUpper(c) || c == '-')) return false;
        if (word.Length >= 2) hasMultiLetter = true;
      }
      return hasMultiLetter;
    }

    // Scan ScenesOut for all-caps speaker-label lines; return canonical CamelCase names.
    private HashSet<string> ExtractSpeakers(string folderOut)
    {
      var speakers = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
      var paths = System.IO.Directory.GetFiles(folderOut);
      foreach (var path in paths)
      {
        var lines = Program.ReadFileAsLines(path);
        foreach (var line in lines)
        {
          if (line.Length == 0) continue;
          var stripped = line.TrimEnd('.', '_', '\'', ' ');
          if (!IsSpeakerLabel(stripped)) continue;
          if (stripped.StartsWith("SCENE") || stripped.StartsWith("ACT")) continue;
          var parts = stripped.Split(new[] { " and ", " AND ", " & " },
                                     StringSplitOptions.RemoveEmptyEntries);
          foreach (var part in parts)
          {
            var name = part.Trim();
            if (name.Length == 0) continue;
            var camel = ExpandActorName(Program.ToCamelCase(name));
            if (!GroupSpeakers.Contains(camel))
              speakers.Add(camel);
          }
        }
      }
      return speakers;
    }

    // Expand abbreviated speaker labels to canonical character names.
    // Handles cases like "PRINCE" → "Prince Henry" in the Henry IV plays.
    public string ExpandActorName(string camelName)
    {
      if (playName == "King Henry IV Part 1" || playName == "King Henry IV Part 2")
      {
        if (camelName.Equals("Prince", StringComparison.OrdinalIgnoreCase))
          return "Prince Henry";
      }
      return camelName;
    }

    // Check if an actor's name (full or abbreviated first-word) appears in camel-case text.
    // Used for Exit/Exeunt line matching where actor names are prose, not all-caps labels.
    private bool ActorAppearsInLine(string actor, string ln)
    {
      var camel = Program.ToCamelCase(actor);
      if (ln.Contains(camel)) return true;
      // Multi-word actor (e.g. "Prince Henry"): also check first word as short form.
      var words = camel.Split(' ');
      if (words.Length > 1 && ln.Contains(words[0])) return true;
      return false;
    }

    // Match speaker-label line against an actor's canonical name.
    // Handles trailing punctuation ("HAMLET._") and joint labels ("CORNELIUS and VOLTEMAND.").
    // Requires the line to be an all-caps speaker label so that stage directions like
    // "Exeunt Voltemand and Cornelius" are never mistaken for a speaker change.
    bool ActorMatchesLine(string actor, string line)
    {
      var ac = Program.ToCamelCase(actor);
      var stripped = line.TrimEnd('.', '_', '\'', ' ');
      if (!IsSpeakerLabel(stripped)) return false;
      var parts = stripped.Split(new[] { " and ", " AND ", " & " },
                                 StringSplitOptions.RemoveEmptyEntries);
      foreach (var part in parts)
      {
        var ln = Program.ToCamelCase(part.Trim());
        if (ac.Equals(ln, StringComparison.OrdinalIgnoreCase)) return true;
        // Accept if expanding the abbreviated label yields the actor's canonical name.
        if (ExpandActorName(ln).Equals(ac, StringComparison.OrdinalIgnoreCase)) return true;
      }
      return false;
    }

    // If the first speaker in a scene has no preceding Enter/Re-enter line, add one.
    private static List<string> InsertMissingOpeningEnter(List<string> lines)
    {
      int firstSpeakerIdx = -1;
      string firstSpeakerLabel = "";
      for (int i = 0; i < lines.Count; i++)
      {
        var stripped = lines[i].Trim().TrimEnd('.', '_', '\'', ' ');
        if (stripped.Length == 0) continue;
        if (stripped.StartsWith("SCENE") || stripped.StartsWith("ACT")) continue;
        if (IsSpeakerLabel(stripped))
        {
          firstSpeakerIdx = i;
          firstSpeakerLabel = stripped;
          break;
        }
      }
      if (firstSpeakerIdx < 0) return lines;

      var camelName = Program.ToCamelCase(firstSpeakerLabel);

      // Check if any prior line is an Enter/Re-enter that mentions this character
      for (int i = 0; i < firstSpeakerIdx; i++)
      {
        var ln = lines[i];
        if ((ln.StartsWith("Enter ") || ln.StartsWith("Re-enter ")) &&
            ln.IndexOf(camelName, StringComparison.OrdinalIgnoreCase) >= 0)
          return lines;   // already has an Enter for them
      }

      // No Enter found — insert one before the speaker label
      var result = new List<string>(lines);
      result.Insert(firstSpeakerIdx, "");
      result.Insert(firstSpeakerIdx, "Enter " + camelName);
      return result;
    }

    List<string> PreprocessLines(List<string> lines, string filename)
    {
      var linesOut = new List<string>();
      foreach (var line in lines)
      {
        if (line.Contains("Falls and dies"))
        {
          linesOut.Add(line.Replace("Falls and dies", "Dies"));
          continue;
        }
        if (line.Contains("[_"))
        {
          linesOut.Add(line.Replace("[_", ""));
          continue;
        }
        if (line.Contains("_]"))
        {
          linesOut.Add(line.Replace("_]", ""));
          continue;
        }
        if (playName == "Hamlet")
        {
          if (filename.Contains("5.2") && line.Contains("They") && line.Contains("play"))
          {
            linesOut.Add(line.Replace("play", "fight"));
            continue;
          }
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
        else if (playName == "King Henry IV Part 1")
        {
          if (line.Contains("DOUGLAS kills SIR WALTER BLUNT"))
          {
            linesOut.Add("Exit SIR WALTER BLUNT");
            linesOut.Add("Enter HOTSPUR");
            continue;
          }
        }
        linesOut.Add(line);
      }
      return InsertMissingOpeningEnter(linesOut);
    }

    public void CopyTrimmedScenes(string folderName)
    {
      var folderOut = folderName + "\\ScenesOut";
      if (!(System.IO.Directory.Exists(folderOut)))
      {
        System.IO.Directory.CreateDirectory(folderOut);
      }

      // Pass 1: preprocess and write scene files.
      var paths = System.IO.Directory.GetFiles(folderName + "\\ScenesIn");
      foreach (var path in paths)
      {
        var filename = System.IO.Path.GetFileName(path).Trim();
        Program.Print("Copying " + filename + ".");
        var lines = Program.ReadFileAsLines(path);
        lines = PreprocessLines(lines, filename);
        Program.Print(lines, Program.PrintMode.FileOnly, folderOut + "\\" + filename, false);
      }

      // Seed Actors directly from the canonical speaker labels found in the output.
      var canonicals = ExtractSpeakers(folderOut);
      foreach (var c in canonicals)
        Actors.Add(c);

      var actorsFilename = folderName + "\\Actors.txt";
      Program.Print("Writing " + actorsFilename + ".");
      Program.Print("Dramatis Personae.", Program.PrintMode.FileOnly, actorsFilename, false);
      Program.Print(Actors.ToList<string>(), Program.PrintMode.FileOnly, actorsFilename, true);
    }

    public bool NotSpeaking(string actor, string line)
    {
      var ln = Program.ToCamelCase(line);

      // "Exit" — bare exit means the current speaker leaves;
      // "Exit, [stage direction]" — same (unnamed, current speaker exits with descriptor);
      // "Exit [Name]" — only the named actor(s) leave.
      if (ln.StartsWith("Exit"))
      {
        var rest = ln.Substring(4).Trim();
        if (rest.Length == 0 || !char.IsUpper(rest[0])) return true;
        return ActorAppearsInLine(actor, ln);
      }

      // "Exeunt" — bare or "Exeunt all" means everyone leaves;
      // "Exeunt all but X" — everyone except X leaves;
      // "Exeunt [Names]" — only the named actors leave.
      if (ln.StartsWith("Exeunt"))
      {
        var rest = ln.Substring(6).Trim();
        if (rest.Length == 0) return true;
        if (rest.StartsWith("All", StringComparison.OrdinalIgnoreCase))
        {
          int butIdx = rest.IndexOf("but ", StringComparison.OrdinalIgnoreCase);
          if (butIdx >= 0)
          {
            var staying = Program.ToCamelCase(rest.Substring(butIdx + 4).Trim());
            if (ActorAppearsInLine(actor, staying)) return false;
          }
          return true;
        }
        return ActorAppearsInLine(actor, ln);
      }

      foreach (var actor2 in Actors)
      {
        if (actor == actor2) { continue; }
        if (ActorMatchesLine(actor2, line)) return true;
      }
      return false;
    }

    bool LineShouldBeAdded(string line)
    {
      if (line.Length == 0) { return false; }
      if (line.StartsWith("Enter ")) { return false; }
      if (line.StartsWith("Re-enter ")) { return false; }
      if (line.StartsWith("Exit ")) { return false; }
      if (line.StartsWith("Exeunt")) { return false; }
      if (SceneViewer.IsBloodRedStageDir(line)) { return false; }
      if (SceneViewer.IsInlineStageDir(line))
      {
        var (_, remainder) = SceneViewer.SplitInlineStageDir(line);
        return remainder.Length > 0; // "Writing." → false; "Within. Hello" → true
      }
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
          bool inScene = false;
          bool theSceneWasStarted = false;
          var filename = System.IO.Path.GetFileName(path).Trim();

          var lines = Program.ReadFileAsLines(path);
          foreach (var line in lines)
          {
            if (theSceneWasStarted && line.StartsWith("Shakespeare homepage")) break;
            if (ActorMatchesLine(actor, line)) { inScene = true; theSceneWasStarted = true; }
            if (line.Contains("Enter ") || line.Contains("Re-enter "))
              if (ActorAppearsInLine(actor, Program.ToCamelCase(line))) inScene = true;
          }
          if (inScene) scenesPresent[actor].Add(filename);
        }
      }

      // Line counts via scene viewer logic (source of truth).
      foreach (var path in paths)
      {
        var filename = System.IO.Path.GetFileName(path).Trim();
        foreach (var kvp in SceneViewer.CountSceneLines(this, path))
        {
          if (kvp.Value > 0 && lineCounts.ContainsKey(kvp.Key))
            lineCounts[kvp.Key][filename] = kvp.Value;
        }
      }
    }

    public void DetectGenders(string folderName)
    {
      var sceneFiles = System.IO.Directory.GetFiles(folderName + "\\ScenesOut");
      foreach (var actor in Actors)
        actorGenders[actor] = GenderDetector.Detect(actor, sceneFiles);
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
      var scenePaths = System.IO.Directory.GetFiles(folderName + "\\ScenesIn");
      foreach (var path in scenePaths)
      {
        var filename = System.IO.Path.GetFileName(path).Trim();
        if (!actorsInEachScene.ContainsKey(filename)) continue;
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
