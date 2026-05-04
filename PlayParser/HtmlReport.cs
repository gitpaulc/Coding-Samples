
using System.Text;
using System.Text.RegularExpressions;

namespace PlayParser
{
    internal static class HtmlReport
    {
        public static string Generate(List<Play> plays)
        {
            var sb = new StringBuilder();
            sb.Append(Header());
            foreach (var play in plays)
                sb.Append(PlaySection(play));
            sb.Append(Footer());
            return sb.ToString();
        }

        // ── ID / slug helpers ─────────────────────────────────────────────────

        private static string Slug(string s) =>
            Regex.Replace(s.ToLowerInvariant(), @"[^a-z0-9]+", "-").Trim('-');

        private static string ActorId(string playSlug, string actor) =>
            $"actor-{playSlug}-{Slug(actor)}";

        private static string SceneId(string playSlug, string filename) =>
            $"scene-{playSlug}-{Slug(Path.GetFileNameWithoutExtension(filename))}";

        // ── Scene filename helpers ────────────────────────────────────────────

        private static int SceneOrder(string filename)
        {
            var name = Path.GetFileNameWithoutExtension(filename);
            if (string.Equals(name, "Induction", StringComparison.OrdinalIgnoreCase)) return 0;
            var parts = name.Split('.');
            if (parts.Length >= 2 && int.TryParse(parts[0], out int act) && int.TryParse(parts[1], out int scene))
                return act * 100 + scene;
            return int.MaxValue;
        }

        private static string SceneLabel(string filename)
        {
            var name = Path.GetFileNameWithoutExtension(filename);
            if (string.Equals(name, "Induction", StringComparison.OrdinalIgnoreCase)) return "Induction";
            var parts = name.Split('.');
            if (parts.Length >= 2 && int.TryParse(parts[0], out int act) && int.TryParse(parts[1], out int scene))
                return $"{act}.{scene}";
            return name;
        }

        private static string SceneLabelLong(string filename)
        {
            var name = Path.GetFileNameWithoutExtension(filename);
            if (string.Equals(name, "Induction", StringComparison.OrdinalIgnoreCase)) return "Induction";
            var parts = name.Split('.');
            if (parts.Length >= 2 && int.TryParse(parts[0], out int act) && int.TryParse(parts[1], out int scene))
                return $"Act {act}, Scene {scene}";
            return name;
        }

        private static string H(string s) => System.Net.WebUtility.HtmlEncode(s);

        // ── Per-play section ─────────────────────────────────────────────────

        private static string PlaySection(Play play)
        {
            string playSlug = Slug(play.playName);
            var totalCounts = play.TotalLineCounts();
            var actorsOrdered = totalCounts
                .OrderByDescending(kvp => kvp.Value)
                .Select(kvp => kvp.Key)
                .ToList();

            int maxLines = actorsOrdered.Count > 0 ? totalCounts[actorsOrdered[0]] : 1;

            var sceneActors = play.GetActorsInEachScene();
            var allScenes = sceneActors.Keys
                .OrderBy(SceneOrder)
                .ToList();

            var sb = new StringBuilder();
            sb.Append($"<section class='play' id='play-{playSlug}'><h2>{H(play.playName)}</h2>");
            sb.Append(BarChart(actorsOrdered, totalCounts, maxLines, playSlug));
            sb.Append(ActorTable(play, actorsOrdered, totalCounts, playSlug));
            sb.Append(SceneTable(allScenes, sceneActors, playSlug));
            sb.Append("</section>");
            return sb.ToString();
        }

        // ── Bar chart ────────────────────────────────────────────────────────

        private static string BarChart(List<string> actors, Dictionary<string, int> totals,
            int maxLines, string playSlug)
        {
            var sb = new StringBuilder();
            sb.Append("<h3>Lines per Actor</h3><div class='barchart'>");
            foreach (var actor in actors)
            {
                int n = totals.TryGetValue(actor, out int v) ? v : 0;
                double pct = maxLines > 0 ? n * 100.0 / maxLines : 0;
                string aid = ActorId(playSlug, actor);
                sb.Append(
                    $"<div class='bar-row'>" +
                    $"<span class='bar-name'><a href='#{aid}'>{H(actor)}</a></span>" +
                    $"<div class='bar-track'>" +
                    $"<div class='bar-fill' style='width:{pct:F1}%'>" +
                    $"<span class='bar-label'>{n}</span>" +
                    $"</div></div></div>");
            }
            sb.Append("</div>");
            return sb.ToString();
        }

        // ── Actor appearances table ───────────────────────────────────────────

        private static string ActorTable(Play play, List<string> actors,
            Dictionary<string, int> totals, string playSlug)
        {
            var sb = new StringBuilder();
            sb.Append(
                "<h3>Scene Appearances</h3>" +
                "<table><thead><tr>" +
                "<th>Actor</th><th>Total Lines</th><th>Scenes (lines)</th>" +
                "</tr></thead><tbody>");

            foreach (var actor in actors)
            {
                string aid = ActorId(playSlug, actor);
                int total = totals.TryGetValue(actor, out int t) ? t : 0;
                var scenes = play.scenesPresent.TryGetValue(actor, out var sl) ? sl : new List<string>();
                var sceneParts = scenes
                    .OrderBy(SceneOrder)
                    .Select(f =>
                    {
                        string label = SceneLabel(f);
                        string sid = SceneId(playSlug, f);
                        string link = $"<a href='#{sid}'>{H(label)}</a>";
                        if (play.lineCounts.TryGetValue(actor, out var sd) && sd.TryGetValue(f, out int lc))
                            return $"{link} ({lc})";
                        return $"{link} (entrance)";
                    });

                sb.Append(
                    $"<tr id='{aid}'><td class='actor-cell'>{H(actor)}</td>" +
                    $"<td class='num-cell'>{total}</td>" +
                    $"<td class='scene-list'>{string.Join(", ", sceneParts)}</td></tr>");
            }

            sb.Append("</tbody></table>");
            return sb.ToString();
        }

        // ── Cast-per-scene table ──────────────────────────────────────────────

        private static string SceneTable(List<string> scenes,
            Dictionary<string, List<string>> sceneActors, string playSlug)
        {
            var sb = new StringBuilder();
            sb.Append(
                "<h3>Cast by Scene</h3>" +
                "<table><thead><tr>" +
                "<th>Scene</th><th>Cast</th>" +
                "</tr></thead><tbody>");

            bool alt = false;
            foreach (var scene in scenes)
            {
                string sid = SceneId(playSlug, scene);
                var actors = sceneActors.TryGetValue(scene, out var al) ? al : new List<string>();
                string rowClass = alt ? " class='alt'" : "";
                var actorLinks = actors.Select(a =>
                    $"<a href='#{ActorId(playSlug, a)}'>{H(a)}</a>");
                sb.Append(
                    $"<tr{rowClass} id='{sid}'>" +
                    $"<td class='scene-cell'>{H(SceneLabelLong(scene))}</td>" +
                    $"<td>{string.Join(", ", actorLinks)}</td></tr>");
                alt = !alt;
            }

            sb.Append("</tbody></table>");
            return sb.ToString();
        }

        // ── HTML shell ───────────────────────────────────────────────────────

        private static string Header() => @"<!DOCTYPE html>
<html lang='en'>
<head>
<meta charset='UTF-8'>
<title>PlayParser Report</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0;}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#c9d1d9;
  line-height:1.65;padding:2rem 1rem 4rem;}
.page{max-width:980px;margin:0 auto;}
header{border-bottom:1px solid #30363d;padding-bottom:1.2rem;margin-bottom:2rem;}
header h1{font-size:1.9rem;font-weight:600;color:#8ab4f8;letter-spacing:-0.4px;}
header p{margin-top:0.3rem;font-size:0.9rem;color:#8b949e;}
section.play{margin-bottom:3.5rem;}
h2{font-size:1.35rem;font-weight:600;color:#e6edf3;margin:2rem 0 1rem;
  padding-bottom:0.5rem;border-bottom:2px solid #21262d;}
h3{font-size:0.95rem;font-weight:600;color:#adbac7;margin:1.6rem 0 0.6rem;
  text-transform:uppercase;letter-spacing:0.04em;}
/* links */
a{color:inherit;text-decoration:none;}
a:hover{text-decoration:underline;opacity:0.85;}
/* bar chart */
.barchart{display:flex;flex-direction:column;gap:4px;margin-bottom:1rem;}
.bar-row{display:flex;align-items:center;gap:8px;}
.bar-name{width:180px;flex-shrink:0;font-size:0.8rem;color:#c9d1d9;
  text-align:right;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.bar-track{flex:1;background:#161b22;border-radius:3px;height:20px;overflow:hidden;}
.bar-fill{height:100%;background:linear-gradient(90deg,#1f6feb,#388bfd);
  border-radius:3px;display:flex;align-items:center;min-width:28px;}
.bar-label{padding:0 6px;font-size:0.72rem;color:#e6edf3;font-weight:600;white-space:nowrap;}
/* tables */
table{width:100%;border-collapse:collapse;font-size:0.82rem;margin-bottom:1rem;}
thead tr{background:#161b22;}
th{padding:7px 10px;text-align:left;color:#8b949e;font-weight:600;
  border-bottom:1px solid #30363d;}
td{padding:6px 10px;border-bottom:1px solid #21262d;vertical-align:top;}
tr.alt td{background:#0d1117;}
.actor-cell{font-weight:600;color:#e6edf3;white-space:nowrap;}
.num-cell{text-align:right;color:#58a6ff;font-weight:600;white-space:nowrap;}
.scene-cell{white-space:nowrap;color:#adbac7;font-weight:600;}
.scene-list{color:#8b949e;font-size:0.78rem;}
</style>
</head>
<body><div class='page'>
<header><h1>PlayParser Report</h1>
<p>Generated " + DateTime.Now.ToString("yyyy-MM-dd HH:mm") + @"</p></header>
";

        private static string Footer() => "</div></body></html>";
    }
}
