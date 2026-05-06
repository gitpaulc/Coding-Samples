
using Microsoft.Web.WebView2.WinForms;
using System.Text;

namespace PlayParser
{
    public class PlayParserForm : Form
    {
        Play? recentScenePlay = null;
        string recentScenePath = "";

        // ── Layout constants ──────────────────────────────────────────────────
        private const int TabStripHeight = 60;
        private const int TabBtnReportWidth = 120;
        private const int TabBtnSceneWidth = 150;
        private const int SceneBarHeight = 72;
        private const int ConsolePanelPercent = 30;

        // ── Fields ────────────────────────────────────────────────────────────
        private ToolStrip toolStrip = null!;
        private ToolStripButton btnRun = null!;
        private ToolStripButton btnSavePdf = null!;
        private SplitContainer split = null!;
        private SplitContainer rightSplit = null!;
        private RichTextBox consoleBox = null!;
        private Label tabBtnReport = null!;
        private Label tabBtnScene = null!;
        private Panel sceneBar = null!;
        private ComboBox actorCombo = null!;
        private Panel colorSwatch = null!;
        private Button actorInfoBtn = null!;
        private WebView2 webView = null!;
        private WebView2 sceneWebView = null!;
        private ComboBox eraCombo = null!;
        private Label authorLabel = null!;
        private ComboBox playCombo = null!;
        private ComboBox sceneCombo = null!;
        private StatusStrip statusStrip = null!;
        private ToolStripStatusLabel statusLabel = null!;
        private bool webViewReady = false;
        private bool sceneWebViewReady = false;
        private int selectedTab = 0;
        private List<Play> lastPlays = new();
        private Dictionary<string, string>? _actorColors;

        // ── Construction ──────────────────────────────────────────────────────
        public PlayParserForm()
        {
            Text = "PlayParser";
            Size = new Size(1440, 900);
            MinimumSize = new Size(900, 600);
            StartPosition = FormStartPosition.CenterScreen;

            try
            {
                var icoPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "shakespeare.ico");
                if (File.Exists(icoPath))
                    Icon = new Icon(icoPath);
            }
            catch { }

            BuildToolStrip();
            BuildStatusStrip();
            BuildSplitContainer();

            Load += async (_, _) => await InitWebView();
        }

        private void BuildToolStrip()
        {
            toolStrip = new ToolStrip { GripStyle = ToolStripGripStyle.Hidden, Padding = new Padding(4, 2, 4, 2) };

            btnRun = new ToolStripButton("▶  Run")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                Font = new Font("Segoe UI", 9.5f, FontStyle.Bold)
            };
            btnRun.Click += BtnRun_Click;

            btnSavePdf = new ToolStripButton("Save PDF")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                Enabled = false
            };
            btnSavePdf.Click += BtnSavePdf_Click;

            toolStrip.Items.Add(btnRun);
            toolStrip.Items.Add(new ToolStripSeparator());
            toolStrip.Items.Add(btnSavePdf);
            Controls.Add(toolStrip);
        }

        private void BuildStatusStrip()
        {
            statusStrip = new StatusStrip();
            statusLabel = new ToolStripStatusLabel("Ready.") { TextAlign = ContentAlignment.MiddleLeft };
            statusStrip.Items.Add(statusLabel);
            Controls.Add(statusStrip);
        }

        private void BuildSplitContainer()
        {
            split = new SplitContainer { Dock = DockStyle.Fill, Orientation = Orientation.Vertical };

            // Left panel: dark console log
            consoleBox = new RichTextBox
            {
                Dock = DockStyle.Fill,
                BackColor = Color.FromArgb(12, 12, 12),
                ForeColor = Color.FromArgb(200, 200, 200),
                Font = new Font("Consolas", 8.5f),
                ReadOnly = true,
                WordWrap = false,
                ScrollBars = RichTextBoxScrollBars.Both
            };
            split.Panel1.Controls.Add(consoleBox);

            // Right panel: rightSplit divides the tab strip (Panel1) from the content (Panel2).
            // Keeping them in separate SplitterPanels guarantees the tab strip can never be
            // overlapped by WebView2's native HWND, which composites above GDI siblings.
            rightSplit = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Horizontal,
                IsSplitterFixed = true,
                SplitterWidth = 1,
                BackColor = Color.FromArgb(48, 54, 61)
            };

            BuildTabStrip(rightSplit.Panel1);
            BuildContentPanel(rightSplit.Panel2);

            split.Panel2.Controls.Add(rightSplit);
            Controls.Add(split);
        }

        private void BuildTabStrip(SplitterPanel panel)
        {
            panel.BackColor = Color.FromArgb(22, 27, 34);
            panel.Paint += (_, e) =>
            {
                var btn = selectedTab == 0 ? tabBtnReport : tabBtnScene;
                using var pen = new Pen(Color.FromArgb(88, 166, 255), 2);
                e.Graphics.DrawLine(pen, btn.Left + 4, panel.Height - 1, btn.Right - 4, panel.Height - 1);
            };

            tabBtnReport = new Label
            {
                Text = "Report",
                AutoSize = false, Width = TabBtnReportWidth, Height = TabStripHeight,
                Left = 0, Top = 0,
                TextAlign = ContentAlignment.BottomCenter,
                Padding = new Padding(0, 0, 0, 10),
                Font = new Font("Segoe UI", 9f),
                Cursor = Cursors.Hand,
                ForeColor = Color.FromArgb(230, 237, 243),
                BackColor = Color.Transparent
            };
            tabBtnReport.Click += (_, _) => SelectTab(0);

            tabBtnScene = new Label
            {
                Text = "Scene Viewer",
                AutoSize = false, Width = TabBtnSceneWidth, Height = TabStripHeight,
                Left = TabBtnReportWidth, Top = 0,
                TextAlign = ContentAlignment.BottomCenter,
                Padding = new Padding(0, 0, 0, 10),
                Font = new Font("Segoe UI", 9f),
                Cursor = Cursors.Hand,
                ForeColor = Color.FromArgb(139, 148, 158),
                BackColor = Color.Transparent
            };
            tabBtnScene.Click += (_, _) => SelectTab(1);

            panel.Controls.Add(tabBtnReport);
            panel.Controls.Add(tabBtnScene);
        }

        private void BuildContentPanel(SplitterPanel panel)
        {
            // Report view
            webView = new WebView2 { Dock = DockStyle.Fill };

            // Scene viewer selector bar (hidden until Scene tab is selected)
            sceneBar = new Panel
            {
                Dock = DockStyle.Top,
                Height = SceneBarHeight,
                BackColor = Color.FromArgb(22, 27, 34),
                Padding = new Padding(6, 0, 6, 0),
                Visible = false
            };

            const int Row1Top = 6;   // combo top in row 1
            const int Row1LblTop = 10; // label top in row 1
            const int Row2 = 36;       // row 2 base offset
            const int Row2Top = Row2 + 6;
            const int Row2LblTop = Row2 + 10;

            // ── Row 1: Era ────────────────────────────────────────────────────
            var lblEra = new Label
            {
                Text = "Era:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = Row1LblTop, Left = 8
            };
            eraCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 110, Top = Row1Top, Left = 40,
                Font = new Font("Segoe UI", 8.5f)
            };
            eraCombo.Items.Add("Classical");
            eraCombo.Items.Add("Renaissance");
            eraCombo.SelectedIndex = 1;

            // ── Row 2: Play + author ──────────────────────────────────────────
            var lblPlay = new Label
            {
                Text = "Play:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = Row2LblTop, Left = 8
            };
            playCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 175, Top = Row2Top, Left = 50,
                Font = new Font("Segoe UI", 8.5f)
            };
            authorLabel = new Label
            {
                Text = "", AutoSize = false, Width = 195, Height = 20,
                Top = Row2LblTop - 1, Left = 234,
                ForeColor = Color.FromArgb(100, 110, 120),
                Font = new Font("Segoe UI", 8.5f, FontStyle.Italic),
                BackColor = Color.Transparent
            };

            // ── Row 2: Scene ──────────────────────────────────────────────────
            var lblScene = new Label
            {
                Text = "Scene:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = Row2LblTop, Left = 438
            };
            sceneCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 160, Top = Row2Top, Left = 484,
                Font = new Font("Segoe UI", 8.5f)
            };

            // ── Row 2: Actor ──────────────────────────────────────────────────
            var lblActor = new Label
            {
                Text = "Actor:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = Row2LblTop, Left = 656
            };
            actorCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 150, Top = Row2Top, Left = 700,
                Font = new Font("Segoe UI", 8.5f)
            };
            actorCombo.Items.Add("None");
            actorCombo.SelectedIndex = 0;

            colorSwatch = new Panel
            {
                Width = 16, Height = 16,
                Left = 856, Top = Row2 + (36 - 16) / 2,
                BackColor = Color.FromArgb(48, 54, 61),
                Visible = false
            };
            actorInfoBtn = new Button
            {
                Text = "Actor Info",
                Size = new Size(82, 22),
                Left = 880, Top = Row2 + (36 - 22) / 2,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(33, 38, 45),
                ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f),
                Cursor = Cursors.Hand,
                Visible = false
            };
            actorInfoBtn.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            actorInfoBtn.Click += ActorInfoBtn_Click;

            eraCombo.SelectedIndexChanged  += EraCombo_Changed;
            playCombo.SelectedIndexChanged += PlayCombo_Changed;
            sceneCombo.SelectedIndexChanged += SceneCombo_Changed;
            actorCombo.SelectedIndexChanged += ActorCombo_Changed;
            sceneBar.Controls.AddRange(new Control[] {
                lblEra, eraCombo, lblPlay, playCombo, authorLabel,
                lblScene, sceneCombo, lblActor, actorCombo, colorSwatch, actorInfoBtn
            });

            sceneWebView = new WebView2 { Dock = DockStyle.Fill, Visible = false };

            // DockStyle.Top (sceneBar) always docks before DockStyle.Fill regardless of add order
            panel.Controls.Add(webView);
            panel.Controls.Add(sceneWebView);
            panel.Controls.Add(sceneBar);
        }

        private void SelectTab(int index)
        {
            if (selectedTab == index) return;
            selectedTab = index;

            tabBtnReport.ForeColor = index == 0 ? Color.FromArgb(230, 237, 243) : Color.FromArgb(139, 148, 158);
            tabBtnScene.ForeColor = index == 1 ? Color.FromArgb(230, 237, 243) : Color.FromArgb(139, 148, 158);
            rightSplit.Panel1.Invalidate();

            webView.Visible = index == 0;
            sceneBar.Visible = index == 1;
            sceneWebView.Visible = index == 1;

            if (index == 1 && !sceneWebViewReady)
                _ = InitSceneWebView();
        }

        // ── WebView initialisation ────────────────────────────────────────────

        private async Task InitWebView()
        {
            split.Panel1MinSize = 200;
            split.Panel2MinSize = 300;
            split.SplitterDistance = Math.Max(200, split.Width * ConsolePanelPercent / 100);
            rightSplit.SplitterDistance = TabStripHeight;

            try
            {
                await webView.EnsureCoreWebView2Async();
                webViewReady = true;
                webView.NavigateToString(SplashHtml());
            }
            catch (Exception ex)
            {
                statusLabel.Text = $"WebView2 error: {ex.Message}";
            }
        }

        private async Task InitSceneWebView()
        {
            try
            {
                await sceneWebView.EnsureCoreWebView2Async();
                sceneWebViewReady = true;
                if (playCombo.SelectedIndex >= 0 && sceneCombo.SelectedItem is SceneItem)
                    SceneCombo_Changed(null, EventArgs.Empty);
                else
                    sceneWebView.NavigateToString(SceneViewerSplashHtml());
            }
            catch (Exception ex)
            {
                statusLabel.Text = $"Scene viewer error: {ex.Message}";
            }
        }

        private static string SplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Press <strong style='color:#e6edf3;'>Run</strong> to analyse the plays.</p>" +
            "</body></html>";

        private static string SceneViewerSplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Run the parser, then select a play and scene.</p>" +
            "</body></html>";

        // ── Console logging ───────────────────────────────────────────────────

        private void AppendLog(string text)
        {
            if (consoleBox.InvokeRequired) { consoleBox.BeginInvoke(() => AppendLog(text)); return; }
            consoleBox.AppendText(text);
            consoleBox.ScrollToCaret();
        }

        // ── Run button ────────────────────────────────────────────────────────

        private async void BtnRun_Click(object? sender, EventArgs e)
        {
            btnRun.Enabled = false;
            btnSavePdf.Enabled = false;
            statusLabel.Text = "Processing...";
            consoleBox.Clear();

            var prevOut = Console.Out;
            Console.SetOut(new GuiTextWriter(AppendLog));

            List<Play> plays;
            try
            {
                plays = await Task.Run(() => Program.RunPlays());
            }
            catch (Exception ex)
            {
                Console.SetOut(prevOut);
                statusLabel.Text = $"Error: {ex.Message}";
                btnRun.Enabled = true;
                return;
            }
            finally
            {
                Console.SetOut(prevOut);
            }

            lastPlays = plays;

            if (webViewReady)
                webView.NavigateToString(HtmlReport.Generate(plays));

            PopulatePlayCombo();

            btnRun.Enabled = true;
            btnSavePdf.Enabled = true;
            statusLabel.Text = "Done.";
        }

        // ── Scene viewer combo logic ──────────────────────────────────────────

        private void PopulatePlayCombo()
        {
            string era = eraCombo.SelectedItem as string ?? "Renaissance";
            playCombo.Items.Clear();
            foreach (var play in lastPlays.Where(p => Play.GetPlayEra(p.playName) == era))
                playCombo.Items.Add(play.playName);
            if (playCombo.Items.Count > 0)
                playCombo.SelectedIndex = 0;
            else
            {
                authorLabel.Text = "";
                sceneCombo.Items.Clear();
            }
        }

        private void EraCombo_Changed(object? sender, EventArgs e) => PopulatePlayCombo();

        private void PlayCombo_Changed(object? sender, EventArgs e)
        {
            sceneCombo.Items.Clear();
            string? playName = playCombo.SelectedItem as string;
            if (playName == null) { authorLabel.Text = ""; return; }

            var play = lastPlays.FirstOrDefault(p => p.playName == playName);
            if (play == null) { authorLabel.Text = ""; return; }

            authorLabel.Text = play.author.Length > 0 ? $"by {play.author}" : "";

            var scenesOut = Path.Combine(Program.GetPlaysFolder(), play.playName, "ScenesOut");
            if (!Directory.Exists(scenesOut)) return;

            var items = Directory.GetFiles(scenesOut, "*.txt")
                .Select(f => new SceneItem(Path.GetFileName(f)))
                .OrderBy(s => SceneViewer.SceneOrder(s.Filename))
                .ToArray<object>();

            sceneCombo.Items.AddRange(items);
            if (sceneCombo.Items.Count > 0)
                sceneCombo.SelectedIndex = 0;
        }

        private void SceneCombo_Changed(object? sender, EventArgs e)
        {
            if (!sceneWebViewReady) return;
            if (playCombo.SelectedItem is not string playName || sceneCombo.SelectedItem is not SceneItem item) return;

            recentScenePlay = lastPlays.FirstOrDefault(p => p.playName == playName);
            if (recentScenePlay == null) return;
            recentScenePath = Path.Combine(Program.GetPlaysFolder(), recentScenePlay.playName, "ScenesOut", item.Filename);
            if (!File.Exists(recentScenePath)) return;

            // Repopulate actor combo without re-triggering a render
            _actorColors = SceneViewer.BuildColorMap(recentScenePlay);
            actorCombo.SelectedIndexChanged -= ActorCombo_Changed;
            actorCombo.Items.Clear();
            actorCombo.Items.Add("None");
            foreach (var actor in SceneViewer.GetSceneActors(recentScenePlay, recentScenePath))
                actorCombo.Items.Add(actor);
            actorCombo.SelectedIndex = 0;
            actorCombo.SelectedIndexChanged += ActorCombo_Changed;
            UpdateColorSwatch();

            sceneWebView.NavigateToString(SceneViewer.RenderScene(recentScenePlay, recentScenePath));
        }

        private void UpdateColorSwatch()
        {
            string? key = actorCombo.SelectedItem as string;
            bool hasActor = key != null && key != "None";
            if (hasActor && _actorColors != null && _actorColors.TryGetValue(key!, out string? hex))
            {
                colorSwatch.BackColor = ColorTranslator.FromHtml(hex);
                colorSwatch.Visible = true;
            }
            else
                colorSwatch.Visible = false;
            actorInfoBtn.Visible = hasActor;
        }

        private void ActorInfoBtn_Click(object? sender, EventArgs e)
        {
            if (playCombo.SelectedItem is not string playName) return;
            string? actorKey = actorCombo.SelectedItem as string;
            if (actorKey == null || actorKey == "None") return;
            var play = lastPlays.FirstOrDefault(p => p.playName == playName);
            if (play == null) return;
            using var dlg = new ActorInfoForm(play, actorKey, recentScenePath);
            dlg.ShowDialog(this);
        }

        public string? GetHighlightActor()
        {
            string? highlight = actorCombo.SelectedItem as string;
            if (highlight == "None") highlight = null;
            // Update highlighting in-place via JS so the scroll position is preserved.
            var actor = (highlight ?? "").Replace("\\", "\\\\").Replace("'", "\\'");
            return actor;
        }

        private async void ActorCombo_Changed(object? sender, EventArgs e)
        {
            if (!sceneWebViewReady) return;
            UpdateColorSwatch();
            var actor = GetHighlightActor();
            await sceneWebView.ExecuteScriptAsync(
                $"typeof setHighlightActor==='function'&&setHighlightActor('{actor}')");
        }

        public void RerenderScene()
        {
            if (recentScenePlay == null || !sceneWebViewReady) return;
            sceneWebView.NavigateToString(SceneViewer.RenderScene(recentScenePlay, recentScenePath));
        }

        // ── Save PDF ──────────────────────────────────────────────────────────

        private async void BtnSavePdf_Click(object? sender, EventArgs e)
        {
            if (!webViewReady) return;

            using var dlg = new SaveFileDialog
            {
                Title = "Save PDF report",
                Filter = "PDF files (*.pdf)|*.pdf",
                FileName = "PlayParserReport.pdf"
            };
            if (dlg.ShowDialog() != DialogResult.OK) return;

            btnSavePdf.Enabled = false;
            statusLabel.Text = "Saving PDF...";
            try
            {
                await webView.CoreWebView2.PrintToPdfAsync(dlg.FileName);
                statusLabel.Text = $"Saved: {dlg.FileName}";
            }
            catch (Exception ex)
            {
                statusLabel.Text = $"PDF error: {ex.Message}";
            }
            finally
            {
                btnSavePdf.Enabled = true;
            }
        }

        // ── Nested types ──────────────────────────────────────────────────────

        private sealed class SceneItem
        {
            public string Filename { get; }
            public SceneItem(string filename) => Filename = filename;
            public override string ToString() => SceneViewer.SceneLabelLong(Filename);
        }
    }

    internal sealed class GuiTextWriter : TextWriter
    {
        private readonly Action<string> _append;
        public GuiTextWriter(Action<string> append) => _append = append;
        public override Encoding Encoding => Encoding.UTF8;
        public override void Write(char value) => _append(value.ToString());
        public override void Write(string? value) => _append(value ?? "");
        public override void WriteLine(string? value) => _append((value ?? "") + "\n");
        public override void WriteLine() => _append("\n");
    }
}
