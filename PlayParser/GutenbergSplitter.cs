using System.Text.RegularExpressions;

namespace PlayParser
{
    /// <summary>
    /// Splits Project Gutenberg source files into per-scene ScenesIn text files,
    /// matching the format expected by PlayParser's CopyTrimmedScenes pipeline.
    ///
    /// Source files live in the Sources/ folder (sibling of the Plays/ folder):
    ///   Sources/hamlet_pg.txt        — PG eBook #1524
    ///   Sources/complete_works.txt   — PG eBook #100  (Henry IV sections)
    ///   Sources/trojan_women_pg.txt  — PG eBook #1914 (Murray translation)
    /// </summary>
    public static class GutenbergSplitter
    {
        // ── Regex patterns ────────────────────────────────────────────────────

        private static readonly Regex ActRe   = new(@"^ACT ([IVX]+)\s*$",  RegexOptions.Compiled);
        private static readonly Regex SceneRe = new(@"^SCENE ([IVX]+)\.",  RegexOptions.Compiled);
        private static readonly Regex SpeakerRe = new(@"^[A-Z][A-Z '\-]*\.$", RegexOptions.Compiled);

        // Trojan Women: "SCENE I." / "SCENE II." / "SCENE V." (may or may not have trailing content)
        private static readonly Regex TwSceneRe = new(@"^SCENE\s+([IVX]+)\.?\s*$", RegexOptions.Compiled);

        // ── Roman numeral helper ──────────────────────────────────────────────

        public static int RomanToInt(string s)
        {
            var vals = new Dictionary<char, int>
            {
                ['I'] = 1, ['V'] = 5, ['X'] = 10, ['L'] = 50
            };
            int total = 0, prev = 0;
            foreach (char ch in s.ToUpper().Reverse())
            {
                int v = vals.TryGetValue(ch, out int val) ? val : 0;
                total += v >= prev ? v : -v;
                prev = v;
            }
            return total;
        }

        // ── Line-level cleaning (mirrors Python clean_line) ───────────────────

        public static string CleanLine(string line)
        {
            string s = line.TrimEnd();

            // 1. [_Exit Name._] → Exit Name
            s = Regex.Replace(s, @"\[_Exit ([^_]+?)\._\]",
                m => "Exit " + m.Groups[1].Value.TrimEnd('.'));
            s = Regex.Replace(s, @"\[_Exit\._\]", "Exit");
            s = Regex.Replace(s, @"\[_Exeunt ([^_]+?)\._\]",
                m => "Exeunt " + m.Groups[1].Value.TrimEnd('.'));
            s = Regex.Replace(s, @"\[_Exeunt\._\]", "Exeunt");

            // 2. Any remaining [_..._] wrapper → unwrap
            s = Regex.Replace(s, @"\[_(.*?)_\]", "$1");

            // 3. Strip trailing period from speaker name lines ("HAMLET." → "HAMLET")
            if (SpeakerRe.IsMatch(s))
                s = s[..^1];

            // 4. Strip trailing period from Enter lines
            if (s.StartsWith("Enter ") && s.EndsWith('.'))
                s = s[..^1];

            // 5. Strip leading whitespace
            s = s.TrimStart();

            return s;
        }

        // ── Scene splitter (mirrors Python split_into_scenes) ─────────────────

        /// <summary>
        /// Walk cleaned play lines; return dict keyed by "1.1", "2.3", "Induction", etc.
        /// </summary>
        public static Dictionary<string, List<string>> SplitIntoScenes(IList<string> lines)
        {
            var scenes = new Dictionary<string, List<string>>();
            string? curKey = null;
            var curLines = new List<string>();
            int? curAct = null;

            void Flush()
            {
                if (curKey == null || curLines.Count == 0) return;
                int start = 0;
                while (start < curLines.Count && curLines[start].Trim() == "") start++;
                int end = curLines.Count;
                while (end > start && curLines[end - 1].Trim() == "") end--;
                scenes[curKey] = curLines.GetRange(start, end - start);
            }

            foreach (string raw in lines)
            {
                string line = raw.TrimEnd();
                string stripped = line.Trim();

                var actMatch = ActRe.Match(stripped);
                if (actMatch.Success)
                {
                    curAct = RomanToInt(actMatch.Groups[1].Value);
                    continue;
                }

                if (stripped == "INDUCTION")
                {
                    Flush();
                    curKey = "Induction";
                    curLines = [];
                    continue;
                }

                var sceneMatch = SceneRe.Match(stripped);
                if (sceneMatch.Success && curAct != null)
                {
                    Flush();
                    int sceneNum = RomanToInt(sceneMatch.Groups[1].Value);
                    curKey = $"{curAct}.{sceneNum}";
                    curLines = [CleanLine(stripped)];
                    continue;
                }

                if (curKey != null)
                    curLines.Add(CleanLine(line));
            }

            Flush();
            return scenes;
        }

        // ── Writer ────────────────────────────────────────────────────────────

        public static void WriteScenes(Dictionary<string, List<string>> scenes, string scenesInDir)
        {
            Directory.CreateDirectory(scenesInDir);
            foreach (var kvp in scenes)
            {
                string path = Path.Combine(scenesInDir, kvp.Key + ".txt");
                File.WriteAllLines(path, kvp.Value, new System.Text.UTF8Encoding(false));
                Console.WriteLine($"  wrote {kvp.Key}.txt  ({kvp.Value.Count} lines)");
            }
        }

        // ── File reader ───────────────────────────────────────────────────────

        private static string[] ReadFile(string path)
        {
            // utf-8-sig: strip BOM if present
            using var sr = new StreamReader(path, new System.Text.UTF8Encoding(false), detectEncodingFromByteOrderMarks: true);
            var lines = new List<string>();
            string? line;
            while ((line = sr.ReadLine()) != null) lines.Add(line);
            return [.. lines];
        }

        // ── Hamlet extractor (PG #1524) ───────────────────────────────────────

        public static Dictionary<string, List<string>> ExtractHamlet(string pgFile)
        {
            var lines = ReadFile(pgFile);
            int start = -1, end = lines.Length;

            for (int i = 0; i < lines.Length; i++)
            {
                if (lines[i].Trim() == "ACT I") { start = i; break; }
            }
            for (int i = 0; i < lines.Length; i++)
            {
                if (lines[i].Contains("*** END OF THE PROJECT GUTENBERG")) { end = i; break; }
            }
            if (start < 0)
                throw new InvalidOperationException("Could not find ACT I in Hamlet file");

            return SplitIntoScenes(lines[start..end]);
        }

        // ── Henry IV extractor (PG Complete Works #100) ───────────────────────

        public static Dictionary<string, List<string>> ExtractFromCompleteWorks(
            string cwFile, string titleMarker, string nextTitleMarker)
        {
            var lines = ReadFile(cwFile);
            int start = -1, end = lines.Length;

            for (int i = 0; i < lines.Length; i++)
            {
                string s = lines[i].Trim();
                if (start < 0 && s == titleMarker)
                {
                    // Advance to first ACT or INDUCTION within the next 300 lines
                    for (int j = i; j < Math.Min(i + 300, lines.Length); j++)
                    {
                        string t = lines[j].Trim();
                        if (ActRe.IsMatch(t) || t == "INDUCTION") { start = j; break; }
                    }
                }
                if (start >= 0 && s == nextTitleMarker && i > start + 10)
                {
                    end = i;
                    break;
                }
            }
            if (start < 0)
                throw new InvalidOperationException($"Could not find play section: {titleMarker}");

            return SplitIntoScenes(lines[start..end]);
        }

        // ── Trojan Women extractor (PG #1914, Murray translation) ─────────────
        //
        // The Murray translation uses "SCENE I." through "SCENE V." (Roman numerals)
        // to delimit the five episodes.  All scenes belong to Act 1.

        public static Dictionary<string, List<string>> ExtractTrojanWomen(string pgFile)
        {
            var allLines = ReadFile(pgFile);
            int start = -1, end = allLines.Length;

            // Locate play start: first "SCENE I." or "Enter Poseidon" after PG header
            for (int i = 0; i < allLines.Length; i++)
            {
                string s = allLines[i].Trim();
                if (TwSceneRe.IsMatch(s) || s == "Enter Poseidon" || s == "PROLOGUE")
                {
                    start = i;
                    break;
                }
            }
            for (int i = 0; i < allLines.Length; i++)
            {
                if (allLines[i].Contains("*** END OF THE PROJECT GUTENBERG")) { end = i; break; }
            }
            if (start < 0)
                throw new InvalidOperationException("Could not find play start in Trojan Women file");

            var playLines = allLines[start..end];
            var scenes = new Dictionary<string, List<string>>();
            string? curKey = null;
            var curLines = new List<string>();
            int sceneCounter = 0;

            void Flush()
            {
                if (curKey == null || curLines.Count == 0) return;
                int s2 = 0;
                while (s2 < curLines.Count && curLines[s2].Trim() == "") s2++;
                int e2 = curLines.Count;
                while (e2 > s2 && curLines[e2 - 1].Trim() == "") e2--;
                if (e2 > s2) scenes[curKey] = curLines.GetRange(s2, e2 - s2);
            }

            foreach (string raw in playLines)
            {
                string line = raw.TrimEnd();
                string stripped = line.Trim();

                // Check for SCENE [Roman]. header
                var m = TwSceneRe.Match(stripped);
                if (m.Success)
                {
                    Flush();
                    sceneCounter = RomanToInt(m.Groups[1].Value);
                    curKey = $"1.{sceneCounter}";
                    curLines = [];
                    continue;
                }

                // If no scene header found yet but we started at "Enter Poseidon" or "PROLOGUE"
                if (curKey == null && (stripped == "Enter Poseidon" || stripped == "PROLOGUE"))
                {
                    sceneCounter = 1;
                    curKey = "1.1";
                    curLines = [];
                    if (stripped != "PROLOGUE")
                        curLines.Add(CleanLine(line));
                    continue;
                }

                if (curKey != null)
                    curLines.Add(CleanLine(line));
            }

            Flush();

            // If no SCENE markers were found and we have only one key, the text is
            // continuous — split on blank-line-separated major structural transitions.
            // This is a fallback for formats that don't have SCENE headers.
            if (scenes.Count <= 1)
                return SplitTrojanWomenByEnter(playLines);

            return scenes;
        }

        // Fallback: split Trojan Women by major "Enter ..." lines that start new episodes
        private static Dictionary<string, List<string>> SplitTrojanWomenByEnter(string[] lines)
        {
            // The five natural scene-break entry points
            string[] sceneOpeners =
            [
                "Enter Poseidon",
                "HECUBA",      // Hecuba's awakening section
                "Enter Andromache",
                "Enter Menelaus",
                "TALTHYBIUS",  // Talthybius bears Astyanax
            ];

            var scenes = new Dictionary<string, List<string>>();
            var curLines = new List<string>();
            int sceneNum = 0;

            bool StartsNewScene(string stripped)
            {
                foreach (string opener in sceneOpeners)
                    if (stripped.StartsWith(opener)) return true;
                return false;
            }

            foreach (string raw in lines)
            {
                string stripped = raw.Trim();
                if (sceneNum < sceneOpeners.Length && StartsNewScene(stripped))
                {
                    if (sceneNum > 0)
                    {
                        int s = 0; while (s < curLines.Count && curLines[s].Trim() == "") s++;
                        int e = curLines.Count; while (e > s && curLines[e - 1].Trim() == "") e--;
                        if (e > s) scenes[$"1.{sceneNum}"] = curLines.GetRange(s, e - s);
                    }
                    sceneNum++;
                    curLines = [CleanLine(raw)];
                    continue;
                }
                if (sceneNum > 0)
                    curLines.Add(CleanLine(raw));
            }

            if (sceneNum > 0)
            {
                int s = 0; while (s < curLines.Count && curLines[s].Trim() == "") s++;
                int e = curLines.Count; while (e > s && curLines[e - 1].Trim() == "") e--;
                if (e > s) scenes[$"1.{sceneNum}"] = curLines.GetRange(s, e - s);
            }

            return scenes;
        }

        // ── Sources folder helper ─────────────────────────────────────────────

        public static string GetSourcesFolder()
        {
            // Walk up from the binary until we find a Sources/ folder, or
            // fall back to the canonical dev layout.
            var dir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
            for (int i = 0; i < 8; i++)
            {
                var candidate = Path.Combine(dir, "Sources");
                if (Directory.Exists(candidate)) return candidate;
                var parent = Path.GetDirectoryName(dir);
                if (parent == null || parent == dir) break;
                dir = parent;
            }
            return Path.GetFullPath(
                Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\PlayParser\Sources"));
        }

        // ── Top-level dispatch ────────────────────────────────────────────────

        /// <summary>
        /// Split one play's source file into ScenesIn text files.
        /// Returns the number of scene files written, or -1 if the source file is missing.
        /// </summary>
        public static int Split(Play.PlayEnum play, string? sourcesFolder = null)
        {
            string sources = sourcesFolder ?? GetSourcesFolder();
            string playsRoot = Program.GetPlaysFolder();
            string playName  = Play.GetPlayName(play);
            string scenesIn  = Path.Combine(playsRoot, playName, "ScenesIn");

            Console.WriteLine($"\n{"=",-60}");
            Console.WriteLine($"Splitting: {playName}");
            Console.WriteLine($"Output:    {scenesIn}");

            Dictionary<string, List<string>> scenes;

            switch (play)
            {
                case Play.PlayEnum.Hamlet:
                {
                    string src = Path.Combine(sources, "hamlet_pg.txt");
                    if (!File.Exists(src))
                    {
                        Console.WriteLine($"  Source file not found: {src}");
                        Console.WriteLine("  Download PG #1524 and save it there.");
                        return -1;
                    }
                    scenes = ExtractHamlet(src);
                    break;
                }

                case Play.PlayEnum.Henry1:
                {
                    string src = Path.Combine(sources, "complete_works.txt");
                    if (!File.Exists(src))
                    {
                        Console.WriteLine($"  Source file not found: {src}");
                        Console.WriteLine("  Download PG #100 and save it there.");
                        return -1;
                    }
                    scenes = ExtractFromCompleteWorks(
                        src,
                        "THE FIRST PART OF KING HENRY THE FOURTH",
                        "THE SECOND PART OF KING HENRY THE FOURTH");
                    break;
                }

                case Play.PlayEnum.Henry2:
                {
                    string src = Path.Combine(sources, "complete_works.txt");
                    if (!File.Exists(src))
                    {
                        Console.WriteLine($"  Source file not found: {src}");
                        Console.WriteLine("  Download PG #100 and save it there.");
                        return -1;
                    }
                    scenes = ExtractFromCompleteWorks(
                        src,
                        "THE SECOND PART OF KING HENRY THE FOURTH",
                        "THE LIFE OF KING HENRY THE FIFTH");
                    break;
                }

                case Play.PlayEnum.TrojanWomen:
                {
                    string src = Path.Combine(sources, "trojan_women_pg.txt");
                    if (!File.Exists(src))
                    {
                        Console.WriteLine($"  Source file not found: {src}");
                        Console.WriteLine("  Download PG #1914 and save it there.");
                        return -1;
                    }
                    scenes = ExtractTrojanWomen(src);
                    break;
                }

                default:
                    throw new ArgumentOutOfRangeException(nameof(play));
            }

            WriteScenes(scenes, scenesIn);
            Console.WriteLine($"  Total scene files written: {scenes.Count}");
            return scenes.Count;
        }

        /// <summary>
        /// Run Split() for every play that has a source file available.
        /// </summary>
        public static void SplitAll(string? sourcesFolder = null)
        {
            for (int i = 0; i < (int)Play.PlayEnum.NumPlays; i++)
                Split((Play.PlayEnum)i, sourcesFolder);
        }
    }
}
