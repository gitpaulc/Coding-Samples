
using System.Net;
using System.Text;

namespace PlayParser
{
    public static class SceneViewer
    {
        public static string enterStr = "Enter";
        public static string exitStr = "Exit";
        public static string exeuntStr = "Exeunt";
        public static string reenterStr = "Re-enter";
        public static HashSet<string> both = new HashSet<string>();

        // ── Scene filename helpers (shared with form) ─────────────────────────

        public static int SceneOrder(string filename)
        {
            var name = Path.GetFileNameWithoutExtension(filename);
            if (string.Equals(name, "Induction", StringComparison.OrdinalIgnoreCase)) return 0;
            var parts = name.Split('.');
            if (parts.Length >= 2 && int.TryParse(parts[0], out int act) && int.TryParse(parts[1], out int scene))
                return act * 100 + scene;
            return int.MaxValue;
        }

        public static string SceneLabelLong(string filename)
        {
            var name = Path.GetFileNameWithoutExtension(filename);
            if (string.Equals(name, "Induction", StringComparison.OrdinalIgnoreCase)) return "Induction";
            var parts = name.Split('.');
            if (parts.Length >= 2 && int.TryParse(parts[0], out int act) && int.TryParse(parts[1], out int scene))
                return $"Act {act}, Scene {scene}";
            return name;
        }
    
        public static bool StartsWithEnterOrExit(string ln)
        {
            return (ln.StartsWith(enterStr + " ") || ln.StartsWith(reenterStr + " ") ||
                    ln.StartsWith(exitStr) || ln.StartsWith(exeuntStr));
        }

        // ── Inline stage direction detection ─────────────────────────────────

        // Words that introduce parenthetical stage directions embedded within speeches
        // (e.g. "Writing.", "Aside. Methinks...", "Cries under the stage. Swear.").
        private static readonly HashSet<string> InlineStageDirStarters =
            new(StringComparer.OrdinalIgnoreCase)
        {
            // Location / delivery
            "Within", "Aside", "Beneath", "Behind", "Above", "Without", "Aloud",
            // 3rd-person action verbs
            "Sings", "Reads", "Drinks", "Retires", "Sleeps", "Cries",
            "Draws", "Falls", "Dies", "Kneels", "Weeps", "Laughs", "Faints",
            // Present participles / gerunds
            "Writing", "Singing", "Advancing", "Reading", "Drawing", "Kneeling",
            "Rising", "Weeping", "Laughing", "Dancing", "Fighting", "Dying",
        };

        // True only when the first word is immediately followed by a period:
        // "Within." or "Within. Hello" → true; "Within Hello" → false.
        private static readonly HashSet<string> BloodRedInlineWords =
            new(StringComparer.OrdinalIgnoreCase) { "Dies", "Falls", "Kills", "Slain" };

        // Stage directions that should render in blood-red.
        public static bool IsBloodRedStageDir(string line)
        {
            string t = line.Trim();
            if (!(t.EndsWith("."))) { return false; }
            if (t.StartsWith("They ", StringComparison.OrdinalIgnoreCase)) return true;
            int spaceIdx = t.IndexOf(' ');
            int dotIdx   = t.IndexOf('.');
            if (dotIdx < 0) return false;
            if (spaceIdx >= 0 && spaceIdx < dotIdx) return false;
            return BloodRedInlineWords.Contains(t[..dotIdx]);
        }

        public static bool IsInlineStageDir(string line)
        {
            string t = line.Trim();
            if (t.Length == 0) return false;
            int spaceIdx = t.IndexOf(' ');
            int dotIdx   = t.IndexOf('.');
            if (dotIdx < 0) return false;
            // A space before the dot means the period isn't right after the first word.
            if (spaceIdx >= 0 && spaceIdx < dotIdx) return false;
            return InlineStageDirStarters.Contains(t[..dotIdx]);
        }

        // Splits "Within. Hello world." into ("Within.", "Hello world.").
        // Caller must only invoke after IsInlineStageDir returns true.
        public static (string StageDir, string Remainder) SplitInlineStageDir(string line)
        {
            string t = line.Trim();
            int dot = t.IndexOf('.');
            return (t[..(dot + 1)], t[(dot + 1)..].Trim());
        }

        // ── Multi-line stage direction joining ────────────────────────────────

        // A stage direction that ends mid-comma continues on the next line(s).
        // Merge those continuation lines so the full direction is one string.
        private static List<string> JoinContinuations(List<string> lines)
        {
            var result = new List<string>();
            int i = 0;
            while (i < lines.Count)
            {
                string line = lines[i];
                if (StartsWithEnterOrExit(line.Trim()))
                {
                    while (line.TrimEnd().EndsWith(',') && i + 1 < lines.Count)
                    {
                        i++;
                        string next = lines[i].Trim();
                        if (next.Length == 0) break;
                        line = line.TrimEnd() + " " + next;
                    }
                }
                result.Add(line);
                i++;
            }
            return result;
        }

        // ── Color assignment ──────────────────────────────────────────────────

        public static Dictionary<string, string> BuildColorMap(Play play) =>
            ColorAssigner.BuildColorMap(play);

        // ── Actor detection ───────────────────────────────────────────────────
        
        enum XtndBool
        {
            False = 0, True = 1, Both = 2, All = 3
        }
        private static bool TryMatchActorXtnd(string trimmed, Play play, out HashSet<string> actorKeys, out XtndBool setType)
        {
            setType = XtndBool.False;
            if (TryMatchActor(trimmed, play, out actorKeys)) { setType = XtndBool.True; return true; }
            actorKeys = new HashSet<string>();
            if (!trimmed.EndsWith('.'))
            {
                var cc = Program.ToCamelCase(trimmed);
                if (cc.Equals("All")) { actorKeys.Add("All"); setType = XtndBool.All; return true; }
                if (cc.Equals("Both")) { foreach(var bothVar in both) { actorKeys.Add(bothVar); } setType = XtndBool.Both; return true; }
                return false;
            }
            var s = trimmed[..^1].TrimEnd();
            var actorKey = Program.ToCamelCase(s);
            if (actorKey.Equals("All")) { actorKeys.Add("All"); setType = XtndBool.All; return true; }
            if (actorKey.Equals("Both")) { foreach(var bothVar in both) { actorKeys.Add(bothVar); } setType = XtndBool.Both; return true; }
            return false;
        }
    
        private static bool TryMatchActor(string trimmed, Play play, out HashSet<string> actorKeys)
        {
            var s = trimmed.EndsWith('.') ? trimmed[..^1].TrimEnd() : trimmed;
            var parts = s.Split(new[] { " and ", " AND ", " & " },
                                 StringSplitOptions.RemoveEmptyEntries);
            actorKeys = new HashSet<string>();
            foreach (var part in parts)
            {
                string actorKey = "";
                if (!TryMatchSomeActor(part, play, out actorKey)) return false;
                if (actorKey.Length == 0) { continue; }
                actorKeys.Add(actorKey);
            }
            return (actorKeys.Count > 0);
        }

        private static bool TryMatchSomeActor(string trimmed, Play play, out string actorKey)
        {
            var s = trimmed.EndsWith('.') ? trimmed[..^1].TrimEnd() : trimmed;
            actorKey = play.ExpandActorName(Program.ToCamelCase(s));
            if (play.Actors.Contains(actorKey)) return true;
            actorKey = "";
            return false;
        }

        // Returns actors in order of first appearance in the scene (speaker lines or stage directions).
        public static List<string> GetSceneActors(Play play, string scenePath)
        {
            var result = new List<string>();
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var raw in JoinContinuations(Program.ReadFileAsLines(scenePath)))
            {
                string line = raw.Trim();
                bool triedMatch = TryMatchActor(line, play, out HashSet<string> keys);
                if (triedMatch)
                {
                    bool shouldContinue = false;
                    foreach (var key in keys)
                    {
                        if (!seen.Add(key)) continue;
                        shouldContinue = true;
                        result.Add(key);
                    }
                    if (shouldContinue) { continue; }
                }
                if (StartsWithEnterOrExit(line))
                {
                    foreach (var actor in play.Actors)
                        if (!seen.Contains(actor) && ActorInStageDir(line, actor))
                        {
                            seen.Add(actor);
                            result.Add(actor);
                        }
                }
            }
            return result;
        }

        // "Exeunt" or "Exeunt all" with no named individuals — everyone on stage exits.
        private static bool IsBareExeunt(string t)
        {
            if (!t.StartsWith(exeuntStr)) return false;
            var rest = t[exeuntStr.Length..].Trim().TrimEnd('.', ',', ';');
            return rest.Length == 0 || string.Equals(rest, "all", StringComparison.OrdinalIgnoreCase);
        }

        private static bool IsExeuntAllBut(string t)
        {
            return t.StartsWith(exeuntStr + " all but");
        }

        // "Exit" with no name — the current speaker exits.
        private static bool IsBareExit(string t)
        {
            if (!t.StartsWith(exitStr) || t.StartsWith(exeuntStr)) return false;
            var rest = t[exitStr.Length..].Trim().TrimEnd('.', ',', ';');
            return rest.Length == 0 || !char.IsUpper(rest[0]);
        }

        // Word-boundary check: is actorKey (or its first word for multi-word names) in the stage dir?
        private static bool ActorInStageDir(string stageDir, string actorKey)
        {
            if (ActorNameInStageDir(stageDir, actorKey)) return true;
            // Multi-word actor (e.g. "Prince Henry"): also try first word as short form.
            int space = actorKey.IndexOf(' ');
            if (space > 0 && ActorNameInStageDir(stageDir, actorKey[..space])) return true;
            return false;
        }

        private static bool ActorNameInStageDir(string stageDir, string name)
        {
            int idx = stageDir.IndexOf(name, StringComparison.OrdinalIgnoreCase);
            if (idx < 0) return false;
            bool leftOk  = idx == 0 || !char.IsLetter(stageDir[idx - 1]);
            bool rightOk = idx + name.Length >= stageDir.Length
                           || !char.IsLetter(stageDir[idx + name.Length]);
            return leftOk && rightOk;
        }

        // ── Line counting (source of truth, mirrors RenderScene logic) ──────────

        public static Dictionary<string, int> CountSceneLines(Play play, string scenePath)
        {
            var counts = new Dictionary<string, int>(StringComparer.Ordinal);
            string? currentActor = null;
            var currentActors = new HashSet<string>(StringComparer.Ordinal);
            var onStage = new HashSet<string>(StringComparer.Ordinal);
            var localBoth = new HashSet<string>(StringComparer.Ordinal);
            bool firstContent = true;

            foreach (var raw in JoinContinuations(Program.ReadFileAsLines(scenePath)))
            {
                string t = raw.Trim();
                if (t.Length == 0) continue;

                if (t.StartsWith("SCENE ") || t.StartsWith("ACT "))
                { currentActor = null; currentActors.Clear(); continue; }

                if (firstContent) { firstContent = false; continue; }

                if (StartsWithEnterOrExit(t))
                {
                    bool entrance = t.StartsWith(enterStr + " ") || t.StartsWith(reenterStr + " ");
                    var exitList = new List<string>();
                    if (!entrance)
                    {
                        if (IsBareExeunt(t)) exitList.AddRange(onStage);
                        else if (IsExeuntAllBut(t)) { foreach (var a in onStage) if (!ActorInStageDir(t, a)) exitList.Add(a); }
                        else if (IsBareExit(t)) { if (currentActor != null) exitList.Add(currentActor); }
                        else { foreach (var a in play.Actors) if (ActorInStageDir(t, a)) exitList.Add(a); }
                    }
                    if (entrance) { foreach (var a in play.Actors) if (ActorInStageDir(t, a)) onStage.Add(a); }
                    else if (IsBareExeunt(t)) onStage.Clear();
                    else if (IsExeuntAllBut(t)) onStage.RemoveWhere(a => !ActorInStageDir(t, a));
                    else if (IsBareExit(t)) { if (currentActor != null) onStage.Remove(currentActor); }
                    else { foreach (var a in play.Actors) if (ActorInStageDir(t, a)) onStage.Remove(a); }
                    if (!entrance && currentActor != null && exitList.Contains(currentActor))
                    { currentActor = null; currentActors.Clear(); }
                    continue;
                }

                if (TryMatchActorXtnd(t, play, out var actorKeys, out var setType) && actorKeys.Count > 0)
                {
                    if (setType == XtndBool.All)
                    { actorKeys.Clear(); foreach (var k in onStage) if (k != currentActor) actorKeys.Add(k); }
                    else if (setType == XtndBool.Both)
                    { actorKeys.Clear(); foreach (var k in localBoth) actorKeys.Add(k); }
                    else if (actorKeys.Count == 2)
                    { localBoth.Clear(); foreach (var k in actorKeys) localBoth.Add(k); }
                    currentActor = actorKeys.Count > 0 ? actorKeys.First() : null;
                    currentActors = new HashSet<string>(actorKeys, StringComparer.Ordinal);
                    foreach (var k in actorKeys) onStage.Add(k);
                    continue;
                }

                if (!IsBloodRedStageDir(t) && !IsInlineStageDir(t) && currentActor != null)
                {
                    foreach (var a in currentActors)
                    {
                        counts.TryGetValue(a, out int c);
                        counts[a] = c + 1;
                    }
                }
            }
            return counts;
        }

        // ── Main render entry point ───────────────────────────────────────────

        // Renders the scene to HTML with data-actor / data-enter / data-exit attributes
        // so that setHighlightActor(actor) can apply annotations without a page reload.
        public static string RenderScene(Play play, string scenePath)
        {
            var lines = JoinContinuations(Program.ReadFileAsLines(scenePath));
            var colorMap = BuildColorMap(play);
            string sceneTitle = SceneLabelLong(Path.GetFileName(scenePath));

            var sb = new StringBuilder();
            var colorMapJson = "{" + string.Join(",", colorMap.Select(kvp => $"\"{kvp.Key}\":\"{kvp.Value}\"")) + "}";
            sb.Append(HtmlHead(sceneTitle, colorMapJson));

            string? currentActor = null;
            var currentActors = new HashSet<string>(StringComparer.Ordinal);
            bool firstContent = true;
            var onStage = new HashSet<string>(StringComparer.Ordinal);

            foreach (var raw in lines)
            {
                string t = raw.Trim();

                if (t.Length == 0)
                {
                    sb.Append("<div class='gap'></div>");
                    continue;
                }

                // Scene / act header
                if (t.StartsWith("SCENE ") || t.StartsWith("ACT "))
                {
                    currentActor = null;
                    currentActors.Clear();
                    sb.Append($"<div class='scene-hdr'>{WebUtility.HtmlEncode(t)}</div>");
                    firstContent = false;
                    continue;
                }

                // First non-blank, non-header line is a location description
                if (firstContent)
                {
                    sb.Append($"<div class='location'>{WebUtility.HtmlEncode(t)}</div>");
                    firstContent = false;
                    continue;
                }

                // Stage directions
                if (StartsWithEnterOrExit(t))
                {
                    bool entrance = t.StartsWith(enterStr + " ") || t.StartsWith(reenterStr + " ");

                    // Compute which actors enter/exit this line (for JS annotation).
                    var enterList = new List<string>();
                    var exitList  = new List<string>();
                    if (entrance)
                    {
                        foreach (var actor in play.Actors)
                            if (ActorInStageDir(t, actor)) enterList.Add(actor);
                    }
                    else if (IsBareExeunt(t))
                        exitList.AddRange(onStage);
                    else if (IsExeuntAllBut(t))
                    {
                        foreach (var actor in onStage)
                            if (!ActorInStageDir(t, actor)) exitList.Add(actor);
                    }
                    else if (IsBareExit(t))
                    {
                        if (currentActor != null) exitList.Add(currentActor);
                    }
                    else
                    {
                        foreach (var actor in play.Actors)
                            if (ActorInStageDir(t, actor)) exitList.Add(actor);
                    }

                    // Update on-stage tracking.
                    if (entrance)
                    {
                        foreach (var a in play.Actors)
                            if (ActorInStageDir(t, a)) onStage.Add(a);
                    }
                    else if (IsBareExeunt(t))
                        onStage.Clear();
                    else if (IsExeuntAllBut(t))
                    {
                        HashSet<string> wasOnStage = new HashSet<string>();
                        foreach (var a in onStage) { wasOnStage.Add(a); }
                        onStage.Clear();
                        foreach (var a in wasOnStage)
                            if (ActorInStageDir(t, a)) onStage.Add(a);
                    }
                    else if (IsBareExit(t))
                    {
                        if (currentActor != null) onStage.Remove(currentActor);
                    }
                    else
                    {
                        foreach (var a in play.Actors)
                            if (ActorInStageDir(t, a)) onStage.Remove(a);
                    }

                    if (!entrance && currentActor != null && exitList.Contains(currentActor))
                    {
                        currentActor = null;
                        currentActors.Clear();
                    }

                    var dataAttrs = new StringBuilder();
                    if (enterList.Count > 0)
                        dataAttrs.Append($" data-enter='{EscapeAttr(string.Join(";", enterList))}'");
                    if (exitList.Count > 0)
                        dataAttrs.Append($" data-exit='{EscapeAttr(string.Join(";", exitList))}'");

                    sb.Append($"<div class='stage-dir'{dataAttrs}>{WebUtility.HtmlEncode(t)}</div>");
                    continue;
                }

                // Actor name (speaker label)
                if (TryMatchActorXtnd(t, play, out HashSet<string> actorKeys, out XtndBool setType) && (actorKeys.Count > 0))
                {
                    var tt = t.EndsWith('.') ? t[..^1].TrimEnd() : t;
                    if (setType == XtndBool.Both) { tt = "BOTH"; }
                    if (setType == XtndBool.All)
                    {
                        tt = "ALL";
                        actorKeys.Clear();
                        foreach (var currentKey in onStage)
                        {
                            if (currentKey == currentActor) { continue; }
                            actorKeys.Add(currentKey);
                        } 
                    }
                    else if (actorKeys.Count == 2)
                    {
                        both.Clear();
                        foreach (var currentKey in actorKeys) { both.Add(currentKey); } 
                    }
                    var actorKey = actorKeys.First();
                    var highlightedActor = Program.GetPlayParserForm()?.GetHighlightActor();
                    if ((highlightedActor != null) && (actorKeys.Count > 1) && (highlightedActor.Length > 0))
                    {
                        if (actorKeys.Contains(highlightedActor))
                        {
                            actorKey = highlightedActor;
                        }
                    }
                    currentActor = actorKey;
                    currentActors = new HashSet<string>(actorKeys, StringComparer.Ordinal);
                    foreach (var actorKey_ in actorKeys) { onStage.Add(actorKey_); }
                    string color = colorMap.TryGetValue(actorKey, out var c) ? c : "#c9d1d9";
                    string actorsAttr = EscapeAttr(string.Join(";", actorKeys));
                    sb.Append($"<div class='actor' data-actors='{actorsAttr}' style='color:{color}'>{WebUtility.HtmlEncode(tt)}</div>");
                    continue;
                }

                // Dialogue or unclassified line
                if (IsBloodRedStageDir(t))
                    sb.Append($"<div class='stage-dir' style='color:#c0392b;font-style:italic'>{WebUtility.HtmlEncode(t)}</div>");
                else if (IsInlineStageDir(t))
                {
                    var (stagePart, remainder) = SplitInlineStageDir(t);
                    sb.Append($"<div class='stage-dir'>{WebUtility.HtmlEncode(stagePart)}</div>");
                    if (remainder.Length > 0)
                    {
                        if (currentActor != null && colorMap.TryGetValue(currentActor, out string? rColor))
                            sb.Append($"<div class='dialogue' data-actor='{EscapeAttr(string.Join(";", currentActors))}' style='color:{rColor}'>{WebUtility.HtmlEncode(remainder)}</div>");
                        else
                            sb.Append($"<div class='stage-dir'>{WebUtility.HtmlEncode(remainder)}</div>");
                    }
                }
                else if (currentActor != null && colorMap.TryGetValue(currentActor, out string? dColor))
                    sb.Append($"<div class='dialogue' data-actor='{EscapeAttr(string.Join(";", currentActors))}' style='color:{dColor}'>{WebUtility.HtmlEncode(t)}</div>");
                else
                    sb.Append($"<div class='stage-dir'>{WebUtility.HtmlEncode(t)}</div>");
            }

            var autoHighlight = Program.GetPlayParserForm()?.GetHighlightActor() ?? "";
            if (autoHighlight.Length > 0)
                sb.Append($"<script>setHighlightActor('{autoHighlight}');</script>");
            sb.Append("</body></html>");
            return sb.ToString();
        }

        private static string EscapeAttr(string s) =>
            s.Replace("&", "&amp;").Replace("'", "&#39;");

        // ── HTML shell ────────────────────────────────────────────────────────

        private static string HtmlHead(string title, string colorMapJson) => $@"<!DOCTYPE html>
<html lang='en'>
<head>
<meta charset='UTF-8'>
<title>{WebUtility.HtmlEncode(title)}</title>
<style>
*,*::before,*::after{{box-sizing:border-box;margin:0;padding:0;}}
body{{font-family:'Georgia','Times New Roman',serif;background:#0d1117;color:#c9d1d9;
  padding:1.5rem 2rem 3rem;line-height:1.75;max-width:820px;}}
.scene-hdr{{font-family:'Segoe UI',sans-serif;font-size:0.78rem;color:#adb8c4;
  letter-spacing:0.1em;text-transform:uppercase;margin:1.8rem 0 0.6rem;}}
.location{{font-style:italic;color:#b8c2cc;margin-bottom:1.2rem;font-size:0.93rem;}}
.gap{{height:0.5rem;}}
.stage-dir{{color:#b8c2cc;font-style:italic;font-size:0.87rem;margin:0.2rem 0;}}
.actor{{font-family:'Segoe UI',sans-serif;font-weight:700;font-size:0.8rem;
  letter-spacing:0.07em;margin-top:1rem;margin-bottom:0.1rem;}}
.dialogue{{padding-left:2rem;font-size:0.94rem;}}
.annot{{float:right;font-family:'Segoe UI',sans-serif;font-size:0.75rem;
  color:#adb8c4;min-width:1.4rem;text-align:right;padding-left:0.5rem;}}
</style>
<script>
var colorMap = {colorMapJson};
function setHighlightActor(actor) {{
  document.querySelectorAll('.annot').forEach(function(el) {{ el.remove(); }});
  document.querySelectorAll('.actor[data-actors]').forEach(function(el) {{
    var actors = el.dataset.actors.split(';');
    var key = (actor && actors.indexOf(actor) >= 0) ? actor : actors[0];
    el.style.color = (colorMap[key] !== undefined) ? colorMap[key] : '#c9d1d9';
  }});
  if (!actor) return;
  var n = 0;
  document.querySelectorAll('.dialogue').forEach(function(el) {{
    var actors = el.dataset.actor ? el.dataset.actor.split(';') : [];
    if (actors.indexOf(actor) >= 0) {{
      var sp = document.createElement('span');
      sp.className = 'annot';
      sp.textContent = ++n;
      el.prepend(sp);
    }}
  }});
  document.querySelectorAll('.stage-dir').forEach(function(el) {{
    var enters = el.dataset.enter ? el.dataset.enter.split(';') : [];
    var exits  = el.dataset.exit  ? el.dataset.exit.split(';')  : [];
    if (enters.indexOf(actor) >= 0) {{
      var sp = document.createElement('span'); sp.className = 'annot'; sp.textContent = '+'; el.prepend(sp);
    }} else if (exits.indexOf(actor) >= 0) {{
      var sp = document.createElement('span'); sp.className = 'annot'; sp.textContent = '×'; el.prepend(sp);
    }}
  }});
}}
</script>
</head>
<body>
<div style='font-family:""Segoe UI"",sans-serif;font-size:0.78rem;color:#adb8c4;
  letter-spacing:0.1em;text-transform:uppercase;margin-bottom:1.5rem;
  padding-bottom:0.6rem;border-bottom:1px solid #21262d;'>{WebUtility.HtmlEncode(title)}</div>
";
    }
}
