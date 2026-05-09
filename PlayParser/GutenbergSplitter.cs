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
    ///   Sources/trojan_women_pg.txt  — PG eBook #35171 (Murray translation)
    /// </summary>
    public static class GutenbergSplitter
    {
        // ── Regex patterns ────────────────────────────────────────────────────

        private static readonly Regex ActRe     = new(@"^ACT ([IVX]+)\s*$",  RegexOptions.Compiled);
        private static readonly Regex SceneRe   = new(@"^SCENE ([IVX]+)\.",  RegexOptions.Compiled | RegexOptions.IgnoreCase);
        private static readonly Regex SpeakerRe = new(@"^[A-Z][A-Z '\-]*\.$", RegexOptions.Compiled);
        private static readonly Regex StarRe    = new(@"^\s*\*\s*\*\s*\*\s*\*\s*\*\s*$", RegexOptions.Compiled);

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
            if (SpeakerRe.IsMatch(s.TrimStart()))
                s = s.TrimStart();
            s = s.TrimEnd();
            if (SpeakerRe.IsMatch(s))
                s = s[..^1];

            // 4. Strip trailing period from Enter lines
            if (s.StartsWith("Enter ") && s.EndsWith('.'))
                s = s[..^1];

            // 5. Strip leading whitespace
            s = s.TrimStart();

            return s;
        }

        // Extended cleaning for Murray's Trojan Women (PG #35171).
        // Handles italic _text_ markers and leading [ artifacts.
        private static string CleanLineTw(string line)
        {
            string s = CleanLine(line);

            // Strip _italic_ markers (lone underscores — not [_..._] which CleanLine handled)
            s = Regex.Replace(s, @"_([^_]+)_", "$1");

            // Strip any residual lone leading underscores
            s = s.TrimStart('_').TrimEnd('_').Trim();

            // Strip a leading bare [ not part of a [_..._] block
            if (s.StartsWith("[") && !s.StartsWith("[_"))
                s = s[1..].TrimStart();

            return s;
        }

        // Pre-join multi-line [_..._] stage-direction blocks into single lines.
        private static List<string> JoinMultiLineBlocks(string[] lines)
        {
            var result = new List<string>();
            string? pending = null;

            foreach (string raw in lines)
            {
                string line = raw.TrimEnd();
                if (pending != null)
                {
                    // Continuation: append (trimming leading indent) until block closes
                    pending += " " + line.TrimStart();
                    if (line.Contains("_]") || line.TrimEnd().EndsWith("_."))
                    {
                        result.Add(pending);
                        pending = null;
                    }
                }
                else if (line.Contains("[_") && !line.Contains("_]") && !line.TrimEnd().EndsWith("_."))
                {
                    pending = line;
                }
                else
                {
                    result.Add(line);
                }
            }

            if (pending != null) result.Add(pending);
            return result;
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

        // ── Author writer ─────────────────────────────────────────────────────

        // Writes Plays/<playName>/Author.txt.
        // Scans the source file for an "Author:" field in the PG header block.
        // Writes "None" if no author can be found in the content.
        private static void WriteAuthor(string playName, string srcFile, string playsRoot)
        {
            string author = "None";

            try
            {
                using var sr = new StreamReader(srcFile, System.Text.Encoding.UTF8, true);
                for (int i = 0; i < 50; i++)
                {
                    var line = sr.ReadLine();
                    if (line == null) break;
                    if (line.StartsWith("Author:", StringComparison.OrdinalIgnoreCase))
                    {
                        author = line["Author:".Length..].Trim();
                        break;
                    }
                    // PG header ends at the *** START *** line
                    if (line.StartsWith("***") && i > 0) break;
                }
            }
            catch { }

            string authorFile = Path.Combine(playsRoot, playName, "Author.txt");
            File.WriteAllText(authorFile, author, new System.Text.UTF8Encoding(false));
            Console.WriteLine($"  wrote Author.txt  ({author})");
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
            using var sr = new StreamReader(path, new System.Text.UTF8Encoding(false),
                detectEncodingFromByteOrderMarks: true);
            var lines = new List<string>();
            string? line;
            while ((line = sr.ReadLine()) != null) lines.Add(line);
            return [.. lines];
        }

        // ── Generic Shakespeare extractor ────────────────────────────────────
        // Works for any individual PG play file that uses ACT/SCENE headers.
        // Finds the first ACT or INDUCTION line as the play start and the PG
        // footer as the end.

        public static Dictionary<string, List<string>> ExtractShakespeare(string pgFile)
        {
            var lines = ReadFile(pgFile);
            int start = -1, end = lines.Length;

            for (int i = 0; i < lines.Length; i++)
            {
                string t = lines[i].Trim();
                if (ActRe.IsMatch(t) || t == "INDUCTION") { start = i; break; }
            }
            for (int i = 0; i < lines.Length; i++)
            {
                if (lines[i].Contains("*** END OF THE PROJECT GUTENBERG")) { end = i; break; }
            }
            if (start < 0)
                throw new InvalidOperationException($"Could not find ACT I or INDUCTION in {Path.GetFileName(pgFile)}");

            return SplitIntoScenes(lines[start..end]);
        }

        // ── Trojan Women extractor (PG #35171, Murray translation) ────────────
        //
        // PG #35171 has no ACT/SCENE headers.  Episodes are separated by
        //   *   *   *   *   *
        // dividers.  Some dividers introduce a chorus ode within a scene
        // (the next non-blank line is "CHORUS."); those do NOT start a new scene.
        // Real scene-break dividers are followed by a stage direction or new entry.
        //
        // Play title "THE TROJAN WOMEN" appears three times; we use the last one
        // (immediately before the opening stage direction).

        public static Dictionary<string, List<string>> ExtractTrojanWomen(string pgFile)
        {
            var allLines = ReadFile(pgFile);
            int start = -1, end = allLines.Length;

            // Find last "THE TROJAN WOMEN" centered title (play text begins after it)
            for (int i = 0; i < allLines.Length; i++)
            {
                if (allLines[i].Trim() == "THE TROJAN WOMEN")
                    start = i + 1;          // keep scanning to find the LAST occurrence
            }
            for (int i = 0; i < allLines.Length; i++)
            {
                if (allLines[i].Contains("*** END OF THE PROJECT GUTENBERG")) { end = i; break; }
            }
            if (start < 0)
                throw new InvalidOperationException("Could not find THE TROJAN WOMEN title in source file");

            // Pre-join multi-line [_..._] stage directions into single lines
            var joined = JoinMultiLineBlocks(allLines[start..end]);

            var scenes = new Dictionary<string, List<string>>();
            var curLines = new List<string>();
            int sceneNum = 1;

            void Flush()
            {
                if (curLines.Count == 0) return;
                int s = 0;
                while (s < curLines.Count && curLines[s].Trim() == "") s++;
                int e = curLines.Count;
                while (e > s && curLines[e - 1].Trim() == "") e--;
                if (e > s) scenes[$"1.{sceneNum}"] = curLines.GetRange(s, e - s);
            }

            for (int i = 0; i < joined.Count; i++)
            {
                string line = joined[i].TrimEnd();
                string stripped = line.Trim();

                if (StarRe.IsMatch(stripped))
                {
                    // Look ahead for first non-blank line after the separator
                    int j = i + 1;
                    while (j < joined.Count && joined[j].Trim() == "") j++;
                    string nextNonBlank = j < joined.Count ? joined[j].Trim() : "";

                    // If that line is "CHORUS." it's a within-scene choral ode — don't split
                    if (nextNonBlank == "CHORUS.")
                    {
                        curLines.Add("");   // blank line separating the chorus section
                        continue;
                    }

                    // Real scene break
                    Flush();
                    sceneNum++;
                    curLines = [];
                    continue;
                }

                curLines.Add(CleanLineTw(line));
            }

            Flush();
            return scenes;
        }

        // ── Sources folder helpers ────────────────────────────────────────────

        public static string GetSourcesEditedFolder()
        {
            var sources = GetSourcesFolder();
            var parent = Path.GetDirectoryName(sources);
            return parent != null
                ? Path.Combine(parent, "SourcesEdited")
                : Path.GetFullPath(Path.Combine(sources, @"..\SourcesEdited"));
        }

        public static string GetSourcesFolder()
        {
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

        // Finds the source file for a play by name (with or without .txt extension).
        private static string? FindSourceFile(string sourcesFolder, string playName)
        {
            foreach (var candidate in new[] { playName, playName + ".txt" })
            {
                var path = Path.Combine(sourcesFolder, candidate);
                if (File.Exists(path)) return path;
            }
            return null;
        }

        // ── Top-level dispatch ────────────────────────────────────────────────

        /// <summary>
        /// Split one play's source file into ScenesIn text files.
        /// Source file is looked up as Sources/&lt;playName&gt; or Sources/&lt;playName&gt;.txt.
        /// Returns the number of scene files written, or -1 if the source file is missing.
        /// </summary>
        public static int Split(string playName, string? sourcesFolder = null)
        {
            string sources   = sourcesFolder ?? GetSourcesFolder();
            string playsRoot = Program.GetPlaysFolder();
            string scenesIn  = Path.Combine(playsRoot, playName, "ScenesIn");

            string? src = FindSourceFile(sources, playName);
            if (src == null)
            {
                Console.WriteLine($"\n{"=",-60}");
                Console.WriteLine($"Splitting: {playName}");
                Console.WriteLine($"  Source not found in {sources}");
                Console.WriteLine($"  Expected: {playName} or {playName}.txt");
                return -1;
            }

            Console.WriteLine($"\n{"=",-60}");
            Console.WriteLine($"Splitting: {playName}");
            Console.WriteLine($"Source:    {src}");
            Console.WriteLine($"Output:    {scenesIn}");

            Dictionary<string, List<string>> scenes = playName == "The Trojan Women"
                ? ExtractTrojanWomen(src)
                : ExtractShakespeare(src);

            Directory.CreateDirectory(Path.Combine(playsRoot, playName));
            WriteAuthor(playName, src, playsRoot);
            WriteScenes(scenes, scenesIn);
            Console.WriteLine($"  Total scene files written: {scenes.Count}");
            return scenes.Count;
        }

        /// <summary>
        /// Run Split() for every *.txt file discovered in the Sources folder.
        /// </summary>
        public static void SplitAll(string? sourcesFolder = null)
        {
            string sources = sourcesFolder ?? GetSourcesFolder();
            if (!Directory.Exists(sources))
            {
                Console.WriteLine($"Sources folder not found: {sources}");
                return;
            }
            foreach (var file in Directory.GetFiles(sources, "*.txt").OrderBy(f => f))
            {
                string playName = Path.GetFileNameWithoutExtension(file);
                Split(playName, sources);
            }
        }
    }
}
