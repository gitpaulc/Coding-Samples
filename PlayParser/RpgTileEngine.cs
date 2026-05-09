
using System.Text.Json;
using System.Text.Json.Serialization;

namespace PlayParser
{
    public static class RpgTileEngine
    {
        // ── Serialised data model (sent to JS as JSON) ────────────────────────

        public class TileChar
        {
            [JsonPropertyName("name")]    public string Name    { get; set; } = "";
            [JsonPropertyName("color")]   public string Color   { get; set; } = "#c9d1d9";
            [JsonPropertyName("x")]       public int    X       { get; set; }
            [JsonPropertyName("y")]       public int    Y       { get; set; }
            [JsonPropertyName("onStage")] public bool   OnStage { get; set; }
        }

        public class TlEvent
        {
            [JsonPropertyName("type")]    public string       Type    { get; set; } = "";
            [JsonPropertyName("speaker")] public string       Speaker { get; set; } = "";
            [JsonPropertyName("text")]    public string       Text    { get; set; } = "";
            [JsonPropertyName("chars")]   public List<string> Chars   { get; set; } = new();
        }

        public class SceneData
        {
            [JsonPropertyName("label")]      public string                   Label      { get; set; } = "";
            [JsonPropertyName("location")]   public string                   Location   { get; set; } = "";
            [JsonPropertyName("mainChar")]   public string                   MainChar   { get; set; } = "";
            [JsonPropertyName("heroChar")]   public string                   HeroChar   { get; set; } = "";
            [JsonPropertyName("gridW")]      public int                      GridW      { get; set; } = 20;
            [JsonPropertyName("gridH")]      public int                      GridH      { get; set; } = 13;
            [JsonPropertyName("chars")]      public List<TileChar>           Chars      { get; set; } = new();
            [JsonPropertyName("lineCounts")] public Dictionary<string, int>  LineCounts { get; set; } = new();
            [JsonPropertyName("genders")]    public Dictionary<string, string> Genders  { get; set; } = new();
            [JsonPropertyName("walls")]      public List<int[]>              Walls      { get; set; } = new();
            [JsonPropertyName("timeline")]   public List<TlEvent>            Timeline   { get; set; } = new();
        }

        // Main first (centre), then ring of secondary positions.
        private static readonly (int x, int y)[] StartPos =
        {
            (5, 3),
            (2, 1), (7, 1), (2, 5), (7, 5),
            (5, 1), (1, 3), (8, 3), (5, 5),
            (3, 2), (6, 2), (3, 4), (6, 4),
        };

        // ── Wall layouts (one set of (x,y) positions per layout) ─────────────

        private static readonly (int x, int y)[][] Layouts =
        {
            // 0: Empty — fallback when connectivity would fail
            Array.Empty<(int x, int y)>(),

            // 1: Corner L-blocks
            new (int x, int y)[] {
                (0,0),(1,0),(2,0),(0,1),(1,1),(0,2),
                (7,0),(8,0),(9,0),(8,1),(9,1),(9,2),
                (0,4),(0,5),(1,5),(0,6),(1,6),(2,6),
                (9,4),(8,5),(9,5),(7,6),(8,6),(9,6),
            },

            // 2: Side alcoves
            new (int x, int y)[] {
                (0,1),(1,1),(0,2),(1,2),
                (0,4),(1,4),(0,5),(1,5),
                (8,1),(9,1),(8,2),(9,2),
                (8,4),(9,4),(8,5),(9,5),
            },

            // 3: Vertical bars (gap at y=3 for passage)
            new (int x, int y)[] {
                (3,0),(3,1),(3,2),(3,4),(3,5),(3,6),
                (6,0),(6,1),(6,2),(6,4),(6,5),(6,6),
            },

            // 4: Horizontal bars (gap at x=5 for passage)
            new (int x, int y)[] {
                (1,2),(2,2),(3,2),(4,2),(6,2),(7,2),(8,2),
                (1,4),(2,4),(3,4),(4,4),(6,4),(7,4),(8,4),
            },

            // 5: Inner quad pillars
            new (int x, int y)[] {
                (2,1),(3,1),(2,2),(3,2),
                (6,1),(7,1),(6,2),(7,2),
                (2,4),(3,4),(2,5),(3,5),
                (6,4),(7,4),(6,5),(7,5),
            },

            // 6: Top/bottom notches + side midpoints
            new (int x, int y)[] {
                (0,0),(1,0),(2,0),(3,0),(6,0),(7,0),(8,0),(9,0),
                (0,6),(1,6),(2,6),(3,6),(6,6),(7,6),(8,6),(9,6),
                (0,2),(0,3),(0,4),(9,2),(9,3),(9,4),
            },
        };

        // ── Public API ───────────────────────────────────────────────────────

        public static List<SceneData> BuildAllScenes(Play play)
        {
            var folder = Path.Combine(Program.GetPlaysFolder(), play.playName, "ScenesOut");
            if (!Directory.Exists(folder)) return new();

            var scenes = Directory.GetFiles(folder, "*.txt")
                .OrderBy(f => SceneViewer.SceneOrder(Path.GetFileName(f)))
                .Select(f => BuildScene(play, f))
                .Where(s => s != null)
                .Select(s => s!)
                .ToList();

            // Assign wall layouts — no two adjacent scenes share the same layout index
            var rng = new Random(Math.Abs(play.playName.GetHashCode()));
            int prevLayout = -1;
            foreach (var scene in scenes)
            {
                int layout;
                int attempts = 0;
                do { layout = rng.Next(1, Layouts.Length); attempts++; }
                while (layout == prevLayout && attempts < 30);
                prevLayout = layout;
                scene.Walls = ComputeWalls(Layouts[layout], scene);
            }

            return scenes;
        }

        public static string ToJson(SceneData data) => JsonSerializer.Serialize(data);

        // ── Wall helpers ─────────────────────────────────────────────────────

        private const int GW = 10;
        private const int GH =  7;
        // X = perimeter of the bounding rectangle
        private const int Perimeter     = 2 * (GW + GH) - 4;
        private const int MinBorderWalls = Perimeter / 4;           // at least X/4 border walls
        private const int MaxBorderWalls = Perimeter - Perimeter / 4; // at least X/4 border clear
        private const int MinTotalWalls  = Perimeter / 2;           // at least X/2 total walls

        private static List<int[]> ComputeWalls((int x, int y)[] layout, SceneData scene)
        {
            // Clear zone: every character tile plus its 8 neighbours
            var clear = new HashSet<(int, int)>();
            foreach (var ch in scene.Chars)
                for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                {
                    int nx = ch.X + dx, ny = ch.Y + dy;
                    if (nx >= 0 && nx < GW && ny >= 0 && ny < GH)
                        clear.Add((nx, ny));
                }

            var walls = new HashSet<(int, int)>(layout.Where(p => !clear.Contains(p)));

            MirrorBorderWalls(walls, clear);

            var positions = scene.Chars.Select(c => (c.X, c.Y)).Distinct().ToList();
            if (!AreAllReachable(walls, positions))
                walls.Clear();

            // Top-up to meet minimums
            if (CountBorder(walls) < MinBorderWalls || walls.Count < MinTotalWalls)
                TopUpWalls(walls, clear, positions, MinBorderWalls, MaxBorderWalls, MinTotalWalls);

            return walls.Select(p => new int[] { p.Item1, p.Item2 }).ToList();
        }

        private static void MirrorBorderWalls(
            HashSet<(int, int)> walls, HashSet<(int, int)> clear)
        {
            var mirrors = new HashSet<(int, int)>();
            foreach (var (x, y) in walls)
            {
                if (x ==      0 && !clear.Contains((GW-1,    y))) mirrors.Add((GW-1,    y));
                if (x == GW - 1 && !clear.Contains((   0,    y))) mirrors.Add((   0,    y));
                if (y ==      0 && !clear.Contains((   x, GH-1))) mirrors.Add((   x, GH-1));
                if (y == GH - 1 && !clear.Contains((   x,    0))) mirrors.Add((   x,    0));
            }
            walls.UnionWith(mirrors);
        }

        private static int CountBorder(HashSet<(int, int)> walls) =>
            walls.Count(p => p.Item1 == 0 || p.Item1 == GW - 1 ||
                             p.Item2 == 0 || p.Item2 == GH - 1);

        private static void TopUpWalls(
            HashSet<(int, int)> walls,
            HashSet<(int, int)> clear,
            List<(int X, int Y)> positions,
            int minBorder, int maxBorder, int minTotal)
        {
            // Candidates: border tiles first (to satisfy minBorder), then interior
            var candidates = new List<(int x, int y)>();
            for (int x = 0; x < GW; x++)
            {
                if (!clear.Contains((x,      0))) candidates.Add((x,      0));
                if (!clear.Contains((x, GH - 1))) candidates.Add((x, GH - 1));
            }
            for (int y = 1; y < GH - 1; y++)
            {
                if (!clear.Contains((     0, y))) candidates.Add((     0, y));
                if (!clear.Contains((GW - 1, y))) candidates.Add((GW - 1, y));
            }
            for (int y = 1; y < GH - 1; y++)
            for (int x = 1; x < GW - 1; x++)
                if (!clear.Contains((x, y))) candidates.Add((x, y));

            foreach (var (cx, cy) in candidates)
            {
                if (CountBorder(walls) >= minBorder && walls.Count >= minTotal) break;
                if (walls.Contains((cx, cy))) continue;

                bool isBorder = cx == 0 || cx == GW - 1 || cy == 0 || cy == GH - 1;

                // Collect the tile and its border mirror(s)
                var toAdd = new HashSet<(int, int)> { (cx, cy) };
                if (cx ==      0 && !clear.Contains((GW - 1, cy))) toAdd.Add((GW - 1, cy));
                if (cx == GW - 1 && !clear.Contains((     0, cy))) toAdd.Add((     0, cy));
                if (cy ==      0 && !clear.Contains((cx, GH - 1))) toAdd.Add((cx, GH - 1));
                if (cy == GH - 1 && !clear.Contains((cx,      0))) toAdd.Add((cx,      0));

                // Don't let this group push border count past maxBorder
                int addedBorder = toAdd.Count(p =>
                    p.Item1 == 0 || p.Item1 == GW - 1 || p.Item2 == 0 || p.Item2 == GH - 1);
                if (isBorder && CountBorder(walls) + addedBorder > maxBorder) continue;

                var test = new HashSet<(int, int)>(walls);
                test.UnionWith(toAdd);
                if (AreAllReachable(test, positions))
                    walls.UnionWith(toAdd);
            }
        }

        private static bool AreAllReachable(
            HashSet<(int x, int y)> walls, List<(int X, int Y)> positions)
        {
            if (positions.Count <= 1) return true;
            var visited = new HashSet<(int, int)>();
            var queue   = new Queue<(int, int)>();
            var start   = (positions[0].X, positions[0].Y);
            queue.Enqueue(start);
            visited.Add(start);
            while (queue.Count > 0)
            {
                var (x, y) = queue.Dequeue();
                foreach (var (dx, dy) in new[] { (0,1),(0,-1),(1,0),(-1,0) })
                {
                    int nx = (x + dx + GW) % GW, ny = (y + dy + GH) % GH;
                    if (walls.Contains((nx, ny))) continue;
                    if (!visited.Add((nx, ny))) continue;
                    queue.Enqueue((nx, ny));
                }
            }
            return positions.All(p => visited.Contains((p.X, p.Y)));
        }

        // ── Scene parsing ─────────────────────────────────────────────────────

        private static SceneData? BuildScene(Play play, string path)
        {
            var rawLines = Program.ReadFileAsLines(path);
            if (rawLines.Count == 0) return null;

            var colorMap  = SceneViewer.BuildColorMap(play);
            string label  = SceneViewer.SceneLabelLong(Path.GetFileName(path));
            string location = "";
            bool firstLine  = true;
            string? curSpeaker = null;
            var onStage    = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var timeline   = new List<TlEvent>();
            var lineCounts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

            foreach (var raw in rawLines)
            {
                string t = raw.Trim();
                if (t.Length == 0) continue;

                if (firstLine)
                {
                    firstLine = false;
                    if (!t.StartsWith("Enter ") && !t.StartsWith("Re-enter "))
                    { location = t; continue; }
                    // Scene opens with Enter — no location line; fall through.
                }
                if (t.StartsWith("SCENE ") || t.StartsWith("ACT ")) continue;

                // ── Enter ────────────────────────────────────────────────────
                if (t.StartsWith("Enter ") || t.StartsWith("Re-enter "))
                {
                    var entering = play.Actors.Where(a => NameInLine(a, t)).ToList();
                    if (entering.Count > 0)
                    {
                        timeline.Add(new TlEvent { Type = "enter", Chars = entering });
                        foreach (var a in entering) onStage.Add(a);
                    }
                    continue;
                }

                // ── Exit / Exeunt ─────────────────────────────────────────────
                if (t.StartsWith("Exit") || t.StartsWith("Exeunt"))
                {
                    var lower = t.ToLowerInvariant();
                    List<string> exiting;

                    if (lower.Contains("all but"))
                    {
                        int bi  = lower.IndexOf("but ") + 4;
                        var rem = bi < t.Length ? t[bi..].Trim() : "";
                        exiting = onStage.Where(a => !NameInLine(a, rem)).ToList();
                    }
                    else if (t == "Exeunt" || lower == "exeunt." || lower == "exeunt all"
                             || lower == "exeunt all.")
                        exiting = onStage.ToList();
                    else if (t.TrimEnd('.') == "Exit" && curSpeaker != null)
                        exiting = new List<string> { curSpeaker };
                    else
                        exiting = play.Actors.Where(a => NameInLine(a, t)).ToList();

                    if (exiting.Count > 0)
                    {
                        timeline.Add(new TlEvent { Type = "exit", Chars = exiting });
                        foreach (var a in exiting) onStage.Remove(a);
                        if (curSpeaker != null &&
                            exiting.Any(a => a.Equals(curSpeaker, StringComparison.OrdinalIgnoreCase)))
                            curSpeaker = null;
                    }
                    continue;
                }

                if (SceneViewer.IsBloodRedStageDir(t)) { curSpeaker = null; continue; }

                // ── Speaker label ─────────────────────────────────────────────
                string? matched = null;
                foreach (var a in play.Actors)
                    if (IsLabel(a, t, play)) { matched = a; break; }
                if (matched != null)
                {
                    curSpeaker = matched;
                    onStage.Add(matched);
                    continue;
                }

                // ── Dialogue line ─────────────────────────────────────────────
                if (curSpeaker != null && !SceneViewer.IsInlineStageDir(t))
                {
                    timeline.Add(new TlEvent { Type = "dialogue", Speaker = curSpeaker, Text = Trunc(t, 160) });
                    lineCounts.TryGetValue(curSpeaker, out int c);
                    lineCounts[curSpeaker] = c + 1;
                }
            }

            if (timeline.Count == 0) return null;

            // Characters on stage before the first dialogue line.
            var initOnStage = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var ev in timeline)
            {
                if (ev.Type == "dialogue") break;
                if (ev.Type == "enter") foreach (var c in ev.Chars) initOnStage.Add(c);
                if (ev.Type == "exit")  foreach (var c in ev.Chars) initOnStage.Remove(c);
            }

            // Hero = true main character of the scene (title actor if present, else most lines).
            string heroChar = play.GetSceneMainCharacter(lineCounts);

            // Initial RPG player = hero if already on stage, else best available on-stage actor.
            string mainChar;
            if (!string.IsNullOrEmpty(heroChar) && initOnStage.Contains(heroChar))
            {
                mainChar = heroChar;
            }
            else
            {
                mainChar = lineCounts
                    .Where(kv => initOnStage.Contains(kv.Key))
                    .OrderByDescending(kv => kv.Value)
                    .Select(kv => kv.Key)
                    .FirstOrDefault()
                    ?? initOnStage.FirstOrDefault()
                    ?? timeline.FirstOrDefault(e => e.Type == "dialogue")?.Speaker
                    ?? play.Actors.FirstOrDefault() ?? "";
            }

            // All participants (speakers + enter/exit actors).
            var allP = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var ev in timeline)
            {
                if (ev.Type == "dialogue") allP.Add(ev.Speaker);
                else foreach (var c in ev.Chars) allP.Add(c);
            }

            // Main goes first so it gets the centre start position.
            var sorted = allP
                .OrderBy(c => c.Equals(mainChar, StringComparison.OrdinalIgnoreCase) ? 0 : 1)
                .ThenBy(c => c)
                .ToList();

            var charList = new List<TileChar>();
            for (int i = 0; i < sorted.Count; i++)
            {
                var name = sorted[i];
                int pi   = Math.Min(i, StartPos.Length - 1);
                charList.Add(new TileChar
                {
                    Name    = name,
                    Color   = colorMap.TryGetValue(name, out var col) ? col : "#c9d1d9",
                    X       = StartPos[pi].x,
                    Y       = StartPos[pi].y,
                    OnStage = initOnStage.Contains(name)
                });
            }

            var genders = new Dictionary<string, string>(StringComparer.Ordinal);
            foreach (var name in allP)
            {
                genders[name] = play.actorGenders.TryGetValue(name, out var g) &&
                                g == ActorGender.Female ? "f" : "m";
            }

            return new SceneData
            {
                Label      = label,
                Location   = location,
                MainChar   = mainChar,
                HeroChar   = heroChar,
                GridW      = GW,
                GridH      = GH,
                Chars      = charList,
                LineCounts = new Dictionary<string, int>(lineCounts, StringComparer.Ordinal),
                Genders    = genders,
                Timeline   = timeline
            };
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        private static bool NameInLine(string actor, string line)
        {
            if (WordInText(actor, line)) return true;
            int sp = actor.IndexOf(' ');
            if (sp > 0 && WordInText(actor[..sp], line)) return true;
            return false;
        }

        private static bool WordInText(string word, string text)
        {
            int i = text.IndexOf(word, StringComparison.OrdinalIgnoreCase);
            if (i < 0) return false;
            bool l = i == 0 || !char.IsLetter(text[i - 1]);
            bool r = i + word.Length >= text.Length || !char.IsLetter(text[i + word.Length]);
            return l && r;
        }

        private static bool IsLabel(string actor, string line, Play play)
        {
            var t = line.TrimEnd('.', '_', '\'', ' ');
            if (!IsAllCaps(t)) return false;
            var parts = t.Split(new[] { " and ", " AND ", " & " },
                                StringSplitOptions.RemoveEmptyEntries);
            foreach (var p in parts)
            {
                var cc = play.ExpandActorName(Program.ToCamelCase(p.Trim()));
                if (cc.Equals(actor, StringComparison.OrdinalIgnoreCase)) return true;
            }
            return false;
        }

        private static bool IsAllCaps(string s)
        {
            bool hasMulti = false;
            foreach (var w in s.Split(' '))
            {
                if (w.Length == 0) continue;
                if (w.Equals("and", StringComparison.OrdinalIgnoreCase) || w == "&") continue;
                if (!w.All(c => char.IsUpper(c) || c == '-')) return false;
                if (w.Length >= 2) hasMulti = true;
            }
            return hasMulti;
        }

        private static string Trunc(string s, int n) =>
            s.Length <= n ? s : s[..n] + "…";
    }
}
