
using System.Net;
using System.Text;

namespace PlayParser
{
    public class RpgEngine
    {
        // ── Data types ───────────────────────────────────────────────────────

        public enum EventKind { Intro, Dialogue, Combat }

        public class RpgEvent
        {
            public EventKind Kind;
            public string Text      = "";
            public string Speaker   = "";   // Dialogue
            public string EnemyName = "";   // Combat
            public int    EnemyMaxHp;
            public int    EnemyHp;
            public int    ThreatLevel = 1;  // 1–3
        }

        public class RpgScene
        {
            public string         Label    = "";
            public string         Location = "";
            public List<RpgEvent> Events   = new();
        }

        // ── State ────────────────────────────────────────────────────────────

        private int _hp = 100;
        public int    PlayerHp   => _hp;
        public int    MaxHp      => 100;
        public string PlayerChar { get; private set; } = "";
        public string PlayName   { get; private set; } = "";
        public List<RpgScene> Scenes { get; private set; } = new();
        public int  SceneIdx { get; private set; }
        public int  EventIdx { get; private set; }
        public bool IsOver   { get; private set; }
        public bool Won      { get; private set; }
        public bool InCombat { get; private set; }

        private readonly List<string> _log = new();
        private readonly Random _rng = new(Environment.TickCount);

        private RpgScene? CurScene => SceneIdx < Scenes.Count ? Scenes[SceneIdx] : null;
        private RpgEvent? CurEvent =>
            CurScene is { } s && EventIdx < s.Events.Count ? s.Events[EventIdx] : null;

        // ── Factory ──────────────────────────────────────────────────────────

        public static RpgEngine Start(Play play, string playerChar)
        {
            var eng = new RpgEngine { PlayerChar = playerChar, PlayName = play.playName };
            eng.Scenes = BuildScenes(play);
            eng._log.Add($"You are {playerChar}. {play.playName} begins.");
            return eng;
        }

        // ── Parsing ──────────────────────────────────────────────────────────

        private static List<RpgScene> BuildScenes(Play play)
        {
            var folder = Path.Combine(Program.GetPlaysFolder(), play.playName, "ScenesOut");
            if (!Directory.Exists(folder)) return new();
            return Directory.GetFiles(folder, "*.txt")
                .OrderBy(f => SceneViewer.SceneOrder(Path.GetFileName(f)))
                .Select(f => BuildScene(play, f))
                .ToList();
        }

        private static RpgScene BuildScene(Play play, string path)
        {
            var scene   = new RpgScene { Label = SceneViewer.SceneLabelLong(Path.GetFileName(path)) };
            bool first  = true;
            string? cur = null;
            bool gotLine = false;
            var used    = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            string introText = "";

            foreach (var raw in Program.ReadFileAsLines(path))
            {
                string t = raw.Trim();
                if (t.Length == 0) continue;
                if (first) { scene.Location = t; first = false; continue; }
                if (t.StartsWith("SCENE ") || t.StartsWith("ACT ")) continue;

                if (t.StartsWith("Enter ") || t.StartsWith("Re-enter "))
                {
                    if (introText.Length == 0) introText = t;
                    continue;
                }
                if (t.StartsWith("Exit") || t.StartsWith("Exeunt")) continue;

                // Death / fight line → combat encounter
                if (SceneViewer.IsBloodRedStageDir(t))
                {
                    FlushIntro(scene, ref introText);
                    string enemy = FindActorInLine(t, play);
                    int threat   = t.StartsWith("They ") ? 3 : 2;
                    scene.Events.Add(new RpgEvent {
                        Kind = EventKind.Combat, Text = t,
                        EnemyName = enemy.Length > 0 ? enemy : "Foe",
                        ThreatLevel = threat, EnemyMaxHp = threat * 30, EnemyHp = threat * 30
                    });
                    cur = null; gotLine = false;
                    continue;
                }

                // Speaker label → track current speaker
                string? matched = null;
                foreach (var a in play.Actors)
                    if (MatchesLabel(a, t, play)) { matched = a; break; }
                if (matched != null) { cur = matched; gotLine = false; continue; }

                // First dialogue line from each unique speaker → Dialogue event
                if (cur != null && !gotLine
                    && !SceneViewer.IsInlineStageDir(t) && !SceneViewer.IsBloodRedStageDir(t)
                    && !used.Contains(cur))
                {
                    FlushIntro(scene, ref introText);
                    used.Add(cur);
                    scene.Events.Add(new RpgEvent {
                        Kind = EventKind.Dialogue, Speaker = cur, Text = Trunc(t, 120)
                    });
                    gotLine = true;
                }
            }

            FlushIntro(scene, ref introText);
            return scene;
        }

        private static void FlushIntro(RpgScene scene, ref string introText)
        {
            if (introText.Length > 0 && scene.Events.Count == 0)
            {
                scene.Events.Insert(0, new RpgEvent { Kind = EventKind.Intro, Text = introText });
                introText = "";
            }
        }

        // ── Actions ──────────────────────────────────────────────────────────

        public void Act(string action)
        {
            if (IsOver) return;
            var ev = CurEvent;
            if (ev == null) { Advance(); return; }
            _log.Clear();

            if (ev.Kind == EventKind.Combat)
                HandleCombat(ev, action);
            else
                Advance();
        }

        private void HandleCombat(RpgEvent ev, string action)
        {
            if (!InCombat) { InCombat = true; _log.Add($"{ev.EnemyName} stands before you!"); return; }

            if (action == "fight")
            {
                int pd = _rng.Next(8, 22);
                ev.EnemyHp = Math.Max(0, ev.EnemyHp - pd);
                _log.Add($"You deal {pd}. {ev.EnemyName}: {ev.EnemyHp}/{ev.EnemyMaxHp} HP.");
                if (ev.EnemyHp <= 0)
                {
                    _log.Add($"{ev.EnemyName} falls!"); InCombat = false; Advance(); return;
                }
                int ed = _rng.Next(5, 8 + ev.ThreatLevel * 4);
                _hp = Math.Max(0, _hp - ed);
                _log.Add($"{ev.EnemyName} deals {ed}. You: {_hp}/{MaxHp} HP.");
                if (_hp == 0) { IsOver = true; Won = false; _log.Add("You have fallen."); }
            }
            else // flee
            {
                int ed = _rng.Next(10, 18);
                _hp = Math.Max(0, _hp - ed);
                _log.Add($"You flee but take {ed} damage. You: {_hp}/{MaxHp} HP.");
                if (_hp == 0) { IsOver = true; Won = false; return; }
                InCombat = false; Advance();
            }
        }

        private void Advance()
        {
            EventIdx++;
            var s = CurScene;
            if (s == null || EventIdx >= s.Events.Count)
            {
                SceneIdx++; EventIdx = 0; InCombat = false;
                if (SceneIdx >= Scenes.Count) { IsOver = true; Won = true; }
            }
        }

        // ── HTML rendering ───────────────────────────────────────────────────

        public string RenderHtml()
        {
            var sb = new StringBuilder();
            sb.Append(Head());

            // Player HP bar
            int hpPct  = PlayerHp * 100 / MaxHp;
            string hpC = hpPct > 50 ? "#2ea043" : hpPct > 20 ? "#d29922" : "#c0392b";
            sb.Append($"<div class='hp-wrap'>" +
                      $"<div class='hp-lbl'>{WebUtility.HtmlEncode(PlayerChar)} — HP {PlayerHp}/{MaxHp}</div>" +
                      $"<div class='track'><div class='fill' style='width:{hpPct}%;background:{hpC}'></div></div>" +
                      $"</div>");

            if (IsOver)
            {
                sb.Append(Won
                    ? $"<div class='result win'>⚔ Victory — you survived <em>{WebUtility.HtmlEncode(PlayName)}</em>.</div>"
                    : "<div class='result lose'>☠ Fallen. The curtain closes.</div>");
                sb.Append(Btn("restart", "Play Again"));
                sb.Append("</body></html>");
                return sb.ToString();
            }

            var scene = CurScene;
            var ev    = CurEvent;
            if (scene == null) { sb.Append("</body></html>"); return sb.ToString(); }

            // Progress indicator
            int total   = Scenes.Count;
            int pctDone = SceneIdx * 100 / Math.Max(1, total);
            sb.Append($"<div class='prog-wrap'>" +
                      $"<div class='prog-lbl'>Scene {SceneIdx + 1} of {total}</div>" +
                      $"<div class='track'><div class='fill' style='width:{pctDone}%;background:#388bfd'></div></div>" +
                      $"</div>");

            sb.Append($"<div class='scene-lbl'>{WebUtility.HtmlEncode(scene.Label)}</div>");
            if (scene.Location.Length > 0)
                sb.Append($"<div class='location'>{WebUtility.HtmlEncode(scene.Location)}</div>");

            if (ev != null) switch (ev.Kind)
            {
                case EventKind.Intro:
                    sb.Append($"<div class='narr'>{WebUtility.HtmlEncode(ev.Text)}</div>");
                    sb.Append(Btn("next", "Continue →"));
                    break;

                case EventKind.Dialogue:
                    sb.Append($"<div class='spkr'>{WebUtility.HtmlEncode(ev.Speaker)}</div>");
                    sb.Append($"<div class='quote'>“{WebUtility.HtmlEncode(ev.Text)}”</div>");
                    sb.Append(Btn("next", "Continue →"));
                    break;

                case EventKind.Combat:
                    int ep  = ev.EnemyMaxHp > 0 ? ev.EnemyHp * 100 / ev.EnemyMaxHp : 0;
                    string ec = ep > 50 ? "#c0392b" : ep > 20 ? "#d29922" : "#555";
                    sb.Append($"<div class='combat-hdr'>⚔ {WebUtility.HtmlEncode(ev.EnemyName)}</div>");
                    sb.Append($"<div class='hp-wrap'>" +
                              $"<div class='hp-lbl'>{WebUtility.HtmlEncode(ev.EnemyName)} HP {ev.EnemyHp}/{ev.EnemyMaxHp}</div>" +
                              $"<div class='track'><div class='fill' style='width:{ep}%;background:{ec}'></div></div>" +
                              $"</div>");
                    sb.Append($"<div class='dir'>{WebUtility.HtmlEncode(ev.Text)}</div>");
                    if (!InCombat)
                        sb.Append(Btn("next", "Engage"));
                    else
                    {
                        sb.Append(Btn("fight", "⚔ Fight"));
                        sb.Append(Btn("flee",  "↩ Flee"));
                    }
                    break;
            }

            if (_log.Count > 0)
            {
                sb.Append("<div class='log'>");
                foreach (var l in _log)
                    sb.Append($"<div>{WebUtility.HtmlEncode(l)}</div>");
                sb.Append("</div>");
            }

            sb.Append("</body></html>");
            return sb.ToString();
        }

        private static string Btn(string action, string label) =>
            $"<button class='btn' onclick=\"chrome.webview.postMessage('{action}')\">{label}</button> ";

        private static string Head() => @"<!DOCTYPE html><html><head><meta charset='UTF-8'><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Georgia',serif;background:#0d1117;color:#c9d1d9;padding:1.5rem 2rem 3rem;max-width:660px;line-height:1.7}
.hp-wrap{margin-bottom:.8rem}
.prog-wrap{margin-bottom:1.2rem}
.hp-lbl,.prog-lbl{font-family:'Segoe UI',sans-serif;font-size:.78rem;color:#8b949e;margin-bottom:3px}
.track{height:7px;background:#21262d;border-radius:4px;overflow:hidden}
.fill{height:100%;border-radius:4px;transition:width .3s}
.scene-lbl{font-family:'Segoe UI',sans-serif;font-size:.76rem;color:#adb8c4;
  letter-spacing:.1em;text-transform:uppercase;margin:.6rem 0 .3rem}
.location{font-style:italic;color:#b8c2cc;font-size:.9rem;margin-bottom:1rem}
.narr{color:#c9d1d9;margin:.4rem 0 .8rem;background:#161b22;padding:.7rem 1rem;
  border-left:3px solid #388bfd;border-radius:0 4px 4px 0}
.spkr{font-family:'Segoe UI',sans-serif;font-weight:700;font-size:.78rem;
  letter-spacing:.07em;margin-top:.7rem;color:#79c0ff}
.quote{padding-left:1.4rem;font-style:italic;margin:.2rem 0 .8rem}
.combat-hdr{font-size:1.05rem;color:#c0392b;font-weight:bold;margin:.5rem 0 .4rem}
.dir{font-style:italic;color:#b8c2cc;font-size:.86rem;margin:.3rem 0 .7rem}
.btn{display:inline-block;margin:.2rem .3rem 0 0;padding:.4rem 1rem;background:#21262d;
  border:1px solid #30363d;color:#c9d1d9;font-size:.88rem;cursor:pointer;
  border-radius:6px;font-family:'Segoe UI',sans-serif}
.btn:hover{background:#30363d;border-color:#8b949e}
.log{margin-top:1rem;padding:.5rem .7rem;background:#161b22;border-radius:4px;
  font-family:'Segoe UI',sans-serif;font-size:.8rem;color:#8b949e;line-height:1.5}
.result{font-size:1.2rem;font-weight:bold;margin-top:2rem;padding:1rem;
  border-radius:6px;text-align:center}
.result.win{color:#2ea043;border:1px solid #2ea043;background:#0a1f0a}
.result.lose{color:#c0392b;border:1px solid #c0392b;background:#1f0a0a}
</style></head><body>
";

        // ── Helpers ──────────────────────────────────────────────────────────

        private static string FindActorInLine(string t, Play play)
        {
            foreach (var a in play.Actors)
                if (t.Contains(a, StringComparison.OrdinalIgnoreCase)) return a;
            int dot = t.IndexOf('.');
            if (dot > 0) { var w = t[..dot].Trim(); if (w.Length < 30) return Program.ToCamelCase(w); }
            return "";
        }

        private static bool MatchesLabel(string actor, string line, Play play)
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

        private static string Trunc(string s, int n) => s.Length <= n ? s : s[..n] + "…";
    }
}
