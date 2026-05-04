
using System.Net;
using System.Text;

namespace PlayParser
{
    internal static class SceneViewer
    {
        private static readonly string[] Palette = {
            "#79c0ff", "#56d364", "#ffa657", "#ff7b72", "#d2a8ff",
            "#e3b341", "#63d0ff", "#f78166", "#aff5b4", "#ffb55a",
            "#a5d6ff", "#85e89d", "#ffc680", "#ff9492", "#b5a9ff",
            "#ffdf5d", "#54aeff", "#6fdd8b", "#ffadad", "#c5a6ff"
        };

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

        // ── Color assignment ──────────────────────────────────────────────────

        public static Dictionary<string, string> BuildColorMap(Play play)
        {
            var map = new Dictionary<string, string>(StringComparer.Ordinal);
            int i = 0;
            foreach (var actor in play.Actors) // SortedSet → alphabetical, stable order
                map[actor] = Palette[i++ % Palette.Length];
            return map;
        }

        // ── Actor detection ───────────────────────────────────────────────────

        private static bool TryMatchActor(string trimmed, Play play, out string actorKey)
        {
            var s = trimmed.EndsWith('.') ? trimmed[..^1].TrimEnd() : trimmed;
            actorKey = Program.ToCamelCase(s);
            if (play.Actors.Contains(actorKey)) return true;
            actorKey = "";
            return false;
        }

        // Returns actors in order of first appearance in the scene (speaker lines or stage directions).
        public static List<string> GetSceneActors(Play play, string scenePath)
        {
            var result = new List<string>();
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var raw in Program.ReadFileAsLines(scenePath))
            {
                string t = raw.Trim();
                if (TryMatchActor(t, play, out string key) && seen.Add(key))
                {
                    result.Add(key);
                    continue;
                }
                if (t.StartsWith("Enter ") || t.StartsWith("Re-enter ") ||
                    t.StartsWith("Exit") || t.StartsWith("Exeunt"))
                {
                    foreach (var actor in play.Actors)
                        if (!seen.Contains(actor) && ActorInStageDir(t, actor))
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
            if (!t.StartsWith("Exeunt")) return false;
            var rest = t["Exeunt".Length..].Trim().TrimEnd('.', ',', ';');
            return rest.Length == 0 || string.Equals(rest, "all", StringComparison.OrdinalIgnoreCase);
        }

        // "Exit" with no name — the current speaker exits.
        private static bool IsBareExit(string t)
        {
            if (!t.StartsWith("Exit") || t.StartsWith("Exeunt")) return false;
            var rest = t["Exit".Length..].Trim().TrimEnd('.', ',', ';');
            return rest.Length == 0;
        }

        // Word-boundary check: is actorKey mentioned in a stage direction?
        private static bool ActorInStageDir(string stageDir, string actorKey)
        {
            int idx = stageDir.IndexOf(actorKey, StringComparison.OrdinalIgnoreCase);
            if (idx < 0) return false;
            bool leftOk  = idx == 0 || !char.IsLetter(stageDir[idx - 1]);
            bool rightOk = idx + actorKey.Length >= stageDir.Length
                           || !char.IsLetter(stageDir[idx + actorKey.Length]);
            return leftOk && rightOk;
        }

        // ── Main render entry point ───────────────────────────────────────────

        public static string RenderScene(Play play, string scenePath, string? highlightActor = null)
        {
            var lines = Program.ReadFileAsLines(scenePath);
            var colorMap = BuildColorMap(play);
            string sceneTitle = SceneLabelLong(Path.GetFileName(scenePath));

            var sb = new StringBuilder();
            sb.Append(HtmlHead(sceneTitle));

            string? currentActor = null;
            bool firstContent = true;
            int lineCount = 0;
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

                // Stage directions — exits clear the current speaker; entrances do not,
                // so dialogue continuing after "Enter/Re-enter X" stays colour-coded.
                if (t.StartsWith("Enter ") || t.StartsWith("Re-enter ") ||
                    t.StartsWith("Exit") || t.StartsWith("Exeunt"))
                {
                    bool entrance = t.StartsWith("Enter ") || t.StartsWith("Re-enter ");

                    // Resolve annotation BEFORE clearing currentActor so bare "Exit"
                    // can still be matched against the departing speaker.
                    string annot = "";
                    if (highlightActor != null)
                    {
                        if (entrance && ActorInStageDir(t, highlightActor))
                            annot = "<span class='annot'>+</span>";
                        else if (!entrance && (ActorInStageDir(t, highlightActor)
                                               || (IsBareExeunt(t) && onStage.Contains(highlightActor))
                                               || (IsBareExit(t) && currentActor == highlightActor)))
                            annot = "<span class='annot'>×</span>";
                    }

                    // Update on-stage tracking so subsequent bare Exeunts are accurate.
                    if (entrance)
                    {
                        foreach (var actor in play.Actors)
                            if (ActorInStageDir(t, actor)) onStage.Add(actor);
                    }
                    else if (IsBareExeunt(t))
                        onStage.Clear();
                    else if (IsBareExit(t))
                    {
                        if (currentActor != null) onStage.Remove(currentActor);
                    }
                    else
                    {
                        foreach (var actor in play.Actors)
                            if (ActorInStageDir(t, actor)) onStage.Remove(actor);
                    }

                    if (t.StartsWith("Exit") || t.StartsWith("Exeunt"))
                        currentActor = null;

                    sb.Append($"<div class='stage-dir'>{annot}{WebUtility.HtmlEncode(t)}</div>");
                    continue;
                }

                // Actor name
                if (TryMatchActor(t, play, out string actorKey))
                {
                    currentActor = actorKey;
                    string color = colorMap.TryGetValue(actorKey, out var c) ? c : "#c9d1d9";
                    sb.Append($"<div class='actor' style='color:{color}'>{WebUtility.HtmlEncode(t)}</div>");
                    continue;
                }

                // Dialogue or unclassified
                if (currentActor != null && colorMap.TryGetValue(currentActor, out string? dColor))
                {
                    string annot = "";
                    if (highlightActor != null && currentActor == highlightActor)
                        annot = $"<span class='annot'>{++lineCount}</span>";
                    sb.Append($"<div class='dialogue' style='color:{dColor}'>{annot}{WebUtility.HtmlEncode(t)}</div>");
                }
                else
                    sb.Append($"<div class='stage-dir'>{WebUtility.HtmlEncode(t)}</div>");
            }

            sb.Append("</body></html>");
            return sb.ToString();
        }

        // ── HTML shell ────────────────────────────────────────────────────────

        private static string HtmlHead(string title) => $@"<!DOCTYPE html>
<html lang='en'>
<head>
<meta charset='UTF-8'>
<title>{WebUtility.HtmlEncode(title)}</title>
<style>
*,*::before,*::after{{box-sizing:border-box;margin:0;padding:0;}}
body{{font-family:'Georgia','Times New Roman',serif;background:#0d1117;color:#c9d1d9;
  padding:1.5rem 2rem 3rem;line-height:1.75;max-width:820px;}}
.scene-hdr{{font-family:'Segoe UI',sans-serif;font-size:0.78rem;color:#6e7681;
  letter-spacing:0.1em;text-transform:uppercase;margin:1.8rem 0 0.6rem;}}
.location{{font-style:italic;color:#8b949e;margin-bottom:1.2rem;font-size:0.93rem;}}
.gap{{height:0.5rem;}}
.stage-dir{{color:#444c56;font-style:italic;font-size:0.87rem;margin:0.2rem 0;}}
.actor{{font-family:'Segoe UI',sans-serif;font-weight:700;font-size:0.8rem;
  letter-spacing:0.07em;margin-top:1rem;margin-bottom:0.1rem;}}
.dialogue{{padding-left:2rem;font-size:0.94rem;}}
.annot{{float:right;font-family:'Segoe UI',sans-serif;font-size:0.75rem;
  color:#8b949e;min-width:1.4rem;text-align:right;padding-left:0.5rem;}}
</style>
</head>
<body>
<div style='font-family:""Segoe UI"",sans-serif;font-size:0.78rem;color:#6e7681;
  letter-spacing:0.1em;text-transform:uppercase;margin-bottom:1.5rem;
  padding-bottom:0.6rem;border-bottom:1px solid #21262d;'>{WebUtility.HtmlEncode(title)}</div>
";
    }
}
