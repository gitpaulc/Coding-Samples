
using Microsoft.Web.WebView2.WinForms;
using System.Text;

namespace PlayParser
{
    public class PlayParserForm : Form
    {
        Play? recentScenePlay = null;
        string recentScenePath = "";
        private static string? _cloudPng;
        private static string LoadCloudPngBase64()
        {
            var asm = System.Reflection.Assembly.GetExecutingAssembly();
            using var stream = asm.GetManifestResourceStream("PlayParser.cloud.png")!;
            using var ms = new System.IO.MemoryStream();
            stream.CopyTo(ms);
            return Convert.ToBase64String(ms.ToArray());
        }

        // ── Layout constants ──────────────────────────────────────────────────
        private const int TabStripHeight    = 60;
        private const int TabBtnReportWidth = 120;
        private const int TabBtnSceneWidth  = 150;
        private const int TabBtnRpgWidth    = 80;
        private const int SceneBarHeight    = 72;
        private const int RpgBarHeight      = 42;
        private const int ConsolePanelPercent = 30;

        // ── Fields ────────────────────────────────────────────────────────────
        private ToolStrip toolStrip = null!;
        private ToolStripButton btnRun = null!;
        private ToolStripButton btnSavePdf = null!;
        private ToolStripButton btnSplit = null!;
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
        // ── RPG tab fields ────────────────────────────────────────────────────
        private Label    tabBtnRpg    = null!;
        private Panel    rpgBar       = null!;
        private ComboBox rpgPlayCombo = null!;
        private Button   btnRpgPrev   = null!;
        private Button   btnRpgNext   = null!;
        private WebView2 rpgWebView   = null!;
        private bool     rpgWebViewReady = false;
        private List<RpgTileEngine.SceneData> _rpgScenes = new();
        private int _rpgSceneIdx;

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

            btnSplit = new ToolStripButton("↓ Split")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                ToolTipText = "Split Gutenberg source files into ScenesIn"
            };
            btnSplit.Click += BtnSplit_Click;

            toolStrip.Items.Add(btnRun);
            toolStrip.Items.Add(new ToolStripSeparator());
            toolStrip.Items.Add(btnSavePdf);
            toolStrip.Items.Add(new ToolStripSeparator());
            toolStrip.Items.Add(btnSplit);
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
                var btn = selectedTab == 0 ? tabBtnReport : selectedTab == 1 ? tabBtnScene : tabBtnRpg;
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

            tabBtnRpg = new Label
            {
                Text = "RPG",
                AutoSize = false, Width = TabBtnRpgWidth, Height = TabStripHeight,
                Left = TabBtnReportWidth + TabBtnSceneWidth, Top = 0,
                TextAlign = ContentAlignment.BottomCenter,
                Padding = new Padding(0, 0, 0, 10),
                Font = new Font("Segoe UI", 9f),
                Cursor = Cursors.Hand,
                ForeColor = Color.FromArgb(139, 148, 158),
                BackColor = Color.Transparent
            };
            tabBtnRpg.Click += (_, _) => SelectTab(2);

            panel.Controls.Add(tabBtnReport);
            panel.Controls.Add(tabBtnScene);
            panel.Controls.Add(tabBtnRpg);
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

            // ── RPG bar ───────────────────────────────────────────────────────
            rpgBar = new Panel
            {
                Dock = DockStyle.Top, Height = RpgBarHeight,
                BackColor = Color.FromArgb(22, 27, 34),
                Padding = new Padding(6, 0, 6, 0), Visible = false
            };
            const int RpgTop = 9, RpgLblTop = 13;
            var lblRpgPlay = new Label {
                Text = "Play:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158), Font = new Font("Segoe UI", 8.5f),
                Top = RpgLblTop, Left = 8 };
            rpgPlayCombo = new ComboBox {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 175, Top = RpgTop, Left = 50, Font = new Font("Segoe UI", 8.5f) };
            var btnRpgStart = new Button {
                Text = "▶ Start", Size = new Size(80, 24), Left = 240, Top = RpgTop,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(33, 38, 45), ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f), Cursor = Cursors.Hand };
            btnRpgStart.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            btnRpgStart.Click += RpgStart_Click;

            btnRpgPrev = new Button {
                Text = "◀ Prev", Size = new Size(75, 24), Left = 334, Top = RpgTop,
                FlatStyle = FlatStyle.Flat, Visible = false,
                BackColor = Color.FromArgb(33, 38, 45), ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f), Cursor = Cursors.Hand };
            btnRpgPrev.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            btnRpgPrev.Click += RpgPrev_Click;

            btnRpgNext = new Button {
                Text = "Next ▶", Size = new Size(75, 24), Left = 418, Top = RpgTop,
                FlatStyle = FlatStyle.Flat, Visible = false,
                BackColor = Color.FromArgb(33, 38, 45), ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f), Cursor = Cursors.Hand };
            btnRpgNext.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            btnRpgNext.Click += RpgNext_Click;

            rpgBar.Controls.AddRange(new Control[] { lblRpgPlay, rpgPlayCombo, btnRpgStart,
                                                     btnRpgPrev, btnRpgNext });

            rpgWebView = new WebView2 { Dock = DockStyle.Fill, Visible = false };

            // DockStyle.Top bars dock before DockStyle.Fill views regardless of Controls.Add order.
            panel.Controls.Add(webView);
            panel.Controls.Add(sceneWebView);
            panel.Controls.Add(sceneBar);
            panel.Controls.Add(rpgWebView);
            panel.Controls.Add(rpgBar);
        }

        private void SelectTab(int index)
        {
            if (selectedTab == index) return;
            selectedTab = index;

            tabBtnReport.ForeColor = index == 0 ? Color.FromArgb(230, 237, 243) : Color.FromArgb(139, 148, 158);
            tabBtnScene.ForeColor  = index == 1 ? Color.FromArgb(230, 237, 243) : Color.FromArgb(139, 148, 158);
            tabBtnRpg.ForeColor    = index == 2 ? Color.FromArgb(230, 237, 243) : Color.FromArgb(139, 148, 158);
            rightSplit.Panel1.Invalidate();

            webView.Visible      = index == 0;
            sceneBar.Visible     = index == 1;
            sceneWebView.Visible = index == 1;
            rpgBar.Visible       = index == 2;
            rpgWebView.Visible   = index == 2;

            if (index == 1 && !sceneWebViewReady) _ = InitSceneWebView();
            if (index == 2 && !rpgWebViewReady)   _ = InitRpgWebView();
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
            PopulateRpgCombos();

            btnRun.Enabled = true;
            btnSavePdf.Enabled = true;
            statusLabel.Text = "Done.";
        }

        // ── Split button ──────────────────────────────────────────────────────

        private async void BtnSplit_Click(object? sender, EventArgs e)
        {
            btnSplit.Enabled = false;
            statusLabel.Text = "Splitting...";
            consoleBox.Clear();

            var prevOut = Console.Out;
            Console.SetOut(new GuiTextWriter(AppendLog));

            try
            {
                await Task.Run(() => GutenbergSplitter.SplitAll());
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
                statusLabel.Text = $"Split error: {ex.Message}";
            }
            finally
            {
                Console.SetOut(prevOut);
                btnSplit.Enabled = true;
                if (statusLabel.Text == "Splitting...")
                    statusLabel.Text = "Split complete.";
            }
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

        // ── RPG tab ───────────────────────────────────────────────────────────

        private void PopulateRpgCombos()
        {
            rpgPlayCombo.Items.Clear();
            foreach (var play in lastPlays)
                rpgPlayCombo.Items.Add(play.playName);
            if (rpgPlayCombo.Items.Count > 0)
                rpgPlayCombo.SelectedIndex = 0;
        }

        private async Task InitRpgWebView()
        {
            try
            {
                await rpgWebView.EnsureCoreWebView2Async();
                rpgWebViewReady = true;
                rpgWebView.CoreWebView2.WebMessageReceived += RpgWebView_MessageReceived;
                rpgWebView.NavigateToString(RpgSplashHtml());
            }
            catch (Exception ex)
            {
                statusLabel.Text = $"RPG view error: {ex.Message}";
            }
        }

        private async void RpgWebView_MessageReceived(object? sender,
            Microsoft.Web.WebView2.Core.CoreWebView2WebMessageReceivedEventArgs e)
        {
            var msg = e.TryGetWebMessageAsString();
            if (msg != "nextScene") return;
            _rpgSceneIdx++;
            if (_rpgSceneIdx < _rpgScenes.Count)
            {
                var json = RpgTileEngine.ToJson(_rpgScenes[_rpgSceneIdx]);
                await rpgWebView.ExecuteScriptAsync($"loadScene({json})");
                UpdateRpgNavButtons();
            }
            else
            {
                await rpgWebView.ExecuteScriptAsync("showVictory()");
                UpdateRpgNavButtons();
            }
        }

        private void RpgStart_Click(object? sender, EventArgs e)
        {
            if (!rpgWebViewReady) return;
            string? playName = rpgPlayCombo.SelectedItem as string;
            if (playName == null) return;
            var play = lastPlays.FirstOrDefault(p => p.playName == playName);
            if (play == null) return;
            _rpgScenes   = RpgTileEngine.BuildAllScenes(play);
            _rpgSceneIdx = 0;
            if (_rpgScenes.Count == 0) return;
            var firstJson = RpgTileEngine.ToJson(_rpgScenes[0]);
            rpgWebView.NavigateToString(GameHtml(firstJson));
            UpdateRpgNavButtons();
        }

        private async void RpgPrev_Click(object? sender, EventArgs e)
        {
            if (_rpgSceneIdx <= 0 || !rpgWebViewReady) return;
            _rpgSceneIdx--;
            var json = RpgTileEngine.ToJson(_rpgScenes[_rpgSceneIdx]);
            await rpgWebView.ExecuteScriptAsync($"loadScene({json})");
            UpdateRpgNavButtons();
        }

        private async void RpgNext_Click(object? sender, EventArgs e)
        {
            if (_rpgSceneIdx >= _rpgScenes.Count - 1 || !rpgWebViewReady) return;
            _rpgSceneIdx++;
            var json = RpgTileEngine.ToJson(_rpgScenes[_rpgSceneIdx]);
            await rpgWebView.ExecuteScriptAsync($"loadScene({json})");
            UpdateRpgNavButtons();
        }

        private void UpdateRpgNavButtons()
        {
            bool active = _rpgScenes.Count > 0;
            btnRpgPrev.Visible = active && _rpgSceneIdx > 0;
            btnRpgNext.Visible = active && _rpgSceneIdx < _rpgScenes.Count - 1;
        }

        private static string RpgSplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Run the parser, select a play, then press " +
            "<strong style='color:#e6edf3;'>▶ Start</strong>.</p></body></html>";

        private static string GameHtml(string firstSceneJson)
        {
            _cloudPng ??= LoadCloudPngBase64();
            return
@"<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;height:100vh;overflow:hidden;user-select:none}
#hud{height:28px;flex-shrink:0;padding:0 12px;background:#161b22;
     border-bottom:1px solid #21262d;display:flex;justify-content:space-between;align-items:center}
#scene-lbl{font-size:.73rem;letter-spacing:.1em;text-transform:uppercase;color:#adb8c4}
#main-lbl{font-size:.73rem;color:#79c0ff}
#location{height:22px;flex-shrink:0;padding:0 12px;font-style:italic;font-size:.8rem;
          color:#8b949e;background:#0d1117;border-bottom:1px solid #161b22;
          display:flex;align-items:center;overflow:hidden;white-space:nowrap}
#main-row{flex:1;min-height:0;display:flex;flex-direction:row}
#canvas-wrap{flex:1;min-width:0;display:flex;align-items:center;
             justify-content:center;overflow:hidden;background:#0d1117}
canvas{display:block}
#dlg{width:220px;flex-shrink:0;padding:16px 14px 12px;
     background:#161b22;border-left:1px solid #21262d;overflow:hidden;
     display:flex;flex-direction:column}
#speaker{font-weight:700;font-size:1.0rem;letter-spacing:.07em;color:#79c0ff;margin-bottom:4px}
#speech{font-style:italic;font-size:1.2rem;color:#e6edf3;line-height:1.55}
#hint{margin-top:10px;font-size:1.0rem;color:#8b949e}
#victory{display:none;position:absolute;inset:0;flex-direction:column;
         align-items:center;justify-content:center;background:#0d1117;
         font-size:1.4rem;color:#2ea043;text-align:center;padding:2rem}
#victory p{margin-top:.5rem;font-size:.95rem;color:#8b949e}
</style>
</head>
<body>
<div id='hud'><span id='scene-lbl'></span><span id='main-lbl'></span></div>
<div id='location'></div>
<div id='main-row'>
<div id='canvas-wrap'><canvas id='game'></canvas></div>
<div id='dlg'>
  <div id='speaker'></div>
  <div id='speech'></div>
  <div id='hint'>Use arrow keys to move into a character to trigger their speech.</div>
</div>
</div>
<div id='victory'><div>&#x2694; You survived the play!</div><p></p></div>
<script>
var TILE=80,GW=10,GH=7;
var canvas=document.getElementById('game');
var ctx=canvas.getContext('2d');
canvas.width=TILE*GW; canvas.height=TILE*GH;

var mainChar='',chars={},timeline=[],tlIdx=0,lineCounts={};
var curLine=null,phase='idle',monoTimer=null;

var cloudImg=new Image(); cloudImg.src='data:image/png;base64,CLOUD_DATA';
var cloudA=0,cloudAng=0,cloudEpi=0,cloudPhaseC='wait',cloudTick=0;
var charSex={},charDir={},charFrame={},walkTick=0;
var arrowVisible=true,arrowTimer=null;
var walls={};
var SKIN='#f5c39a',HAIR_M='#2d1a08',HAIR_F='#8b4513',
    SHOE='#2a1a0a',PANT='#2d3a5c',BELT='#5a4020';

function showVictory(){
  var v=document.getElementById('victory');
  v.style.display='flex';
  document.querySelector('#victory p').textContent='All scenes complete.';
}

function loadScene(data){
  if(monoTimer){clearTimeout(monoTimer);monoTimer=null;}
  if(arrowTimer){clearInterval(arrowTimer);arrowTimer=null;} arrowVisible=true;
  mainChar=data.mainChar; lineCounts=data.lineCounts||{};
  timeline=data.timeline; tlIdx=0; curLine=null; phase='idle';
  chars={}; charSex=data.genders||{}; walkTick=0;
  walls={};
  if(data.walls){for(var wi=0;wi<data.walls.length;wi++)
    walls[data.walls[wi][0]+','+data.walls[wi][1]]=true;}
  for(var i=0;i<data.chars.length;i++){
    var c=data.chars[i];
    chars[c.name]={x:c.x,y:c.y,onStage:c.onStage,color:c.color};
    charDir[c.name]='down'; charFrame[c.name]=0;
  }
  document.getElementById('scene-lbl').textContent=data.label;
  document.getElementById('location').textContent=data.location;
  document.getElementById('main-lbl').textContent='You: '+mainChar;
  setSpeech('','');
  document.getElementById('hint').textContent='Move into a character to trigger their speech.';
  step(); render();
}

function setSpeech(spkr,txt){
  document.getElementById('speaker').textContent=spkr;
  document.getElementById('speech').textContent=txt;
}

function step(){
  while(tlIdx<timeline.length){
    var ev=timeline[tlIdx];

    if(ev.type==='enter'){
      for(var i=0;i<ev.chars.length;i++) if(chars[ev.chars[i]]) chars[ev.chars[i]].onStage=true;
      tlIdx++; continue;
    }

    if(ev.type==='exit'){
      for(var i=0;i<ev.chars.length;i++){
        var c=ev.chars[i];
        if(chars[c]) chars[c].onStage=false;
        if(c===mainChar){
          mainChar=newMain(c);
          document.getElementById('main-lbl').textContent='You: '+mainChar;
        }
      }
      tlIdx++; continue;
    }

    if(ev.type==='dialogue'){
      curLine=ev;
      setSpeech(ev.speaker, '“'+ev.text+'”');
      var spkrChar=chars[ev.speaker];
      var spkrOnStage=spkrChar&&spkrChar.onStage;

      if(ev.speaker===mainChar){
        stopArrow();
        phase='mono';
        document.getElementById('hint').textContent=mainChar+' (monologue)';
        monoTimer=setTimeout(function(){tlIdx++;step();render();},2200);
      } else if(!spkrOnStage){
        stopArrow();
        phase='mono';
        document.getElementById('hint').textContent='(off-stage)';
        monoTimer=setTimeout(function(){tlIdx++;step();render();},1200);
      } else {
        // Player must walk into the speaker
        phase='wait';
        document.getElementById('hint').textContent='Walk into '+ev.speaker;
        if(!arrowTimer) arrowTimer=setInterval(function(){arrowVisible=!arrowVisible;render();},1000);
      }
      break;
    }

    tlIdx++;
  }

  if(tlIdx>=timeline.length){
    stopArrow();
    phase='done';
    document.getElementById('hint').textContent='Scene complete — loading next…';
    setTimeout(function(){chrome.webview.postMessage('nextScene');},1000);
  }
}

function newMain(exited){
  var best='',bestN=-1,keys=Object.keys(chars);
  for(var i=0;i<keys.length;i++){
    var n=keys[i],c=chars[n];
    if(!c.onStage||n===exited) continue;
    var cnt=lineCounts[n]||0;
    if(cnt>bestN){bestN=cnt;best=n;}
  }
  if(!best) for(var i=0;i<keys.length;i++) if(chars[keys[i]].onStage){best=keys[i];break;}
  return best;
}

function move(dx,dy){
  if(phase==='done'||phase==='mono') return;
  var m=chars[mainChar]; if(!m) return;
  var nx=(m.x+dx+GW)%GW;
  var ny=(m.y+dy+GH)%GH;

  // Always update facing direction
  if(dx===1) charDir[mainChar]='right';
  else if(dx===-1) charDir[mainChar]='left';
  else if(dy===1) charDir[mainChar]='down';
  else charDir[mainChar]='up';

  // Dialogue trigger fires on bump attempt, before blocking
  if(phase==='wait'){
    var ev=timeline[tlIdx];
    if(ev&&ev.type==='dialogue'){
      var spkr=chars[ev.speaker];
      if(spkr&&spkr.onStage&&nx===spkr.x&&ny===spkr.y){
        walkTick++; charFrame[mainChar]=[0,1,2,1][walkTick%4];
        render(); tlIdx++; step(); render(); return;
      }
    }
  }

  // Block movement into wall or occupied tile
  var blocked=!!walls[nx+','+ny];
  if(!blocked){
    var keys=Object.keys(chars);
    for(var i=0;i<keys.length;i++){
      var n=keys[i]; if(n===mainChar) continue;
      var oc=chars[n];
      if(oc.onStage&&oc.x===nx&&oc.y===ny){blocked=true;break;}
    }
  }

  if(!blocked){ m.x=nx; m.y=ny; }
  walkTick++; charFrame[mainChar]=[0,1,2,1][walkTick%4];
  render();
}

function render(){
  ctx.fillStyle='#0d1117'; ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle='#161b22'; ctx.lineWidth=1;
  for(var gx=0;gx<=GW;gx++){
    ctx.beginPath();ctx.moveTo(gx*TILE,0);ctx.lineTo(gx*TILE,canvas.height);ctx.stroke();
  }
  for(var gy=0;gy<=GH;gy++){
    ctx.beginPath();ctx.moveTo(0,gy*TILE);ctx.lineTo(canvas.width,gy*TILE);ctx.stroke();
  }

  var wkeys=Object.keys(walls);
  for(var wi=0;wi<wkeys.length;wi++){
    var wp=wkeys[wi].split(',');
    var wrx=parseInt(wp[0])*TILE,wry=parseInt(wp[1])*TILE;
    ctx.fillStyle='#1c2128'; ctx.fillRect(wrx,wry,TILE,TILE);
    ctx.fillStyle='#2d333b'; ctx.fillRect(wrx+2,wry+2,TILE-4,TILE-4);
    ctx.fillStyle='#373e47'; ctx.fillRect(wrx+2,wry+2,TILE-4,2);
                             ctx.fillRect(wrx+2,wry+2,2,TILE-4);
  }

  var nextSpkr=(phase==='wait'&&curLine)?curLine.speaker:null;
  var keys=Object.keys(chars);
  for(var i=0;i<keys.length;i++){
    var name=keys[i],c=chars[name];
    if(!c.onStage) continue;
    var cx=c.x*TILE+TILE/2, cy=c.y*TILE+TILE/2;
    var isMain=(name===mainChar), isNext=(name===nextSpkr);
    // Shadow glow beneath feet
    if(isMain||isNext){
      ctx.save(); ctx.globalAlpha=.5;
      ctx.fillStyle=isMain?'#ffffff':'#ffa657';
      ctx.beginPath(); ctx.ellipse(cx,cy+28,18,6,0,0,Math.PI*2); ctx.fill();
      ctx.restore();
    }
    var sex=charSex[name]||'m';
    var dir=isMain?(charDir[mainChar]||'down'):'down';
    var frm=isMain?(charFrame[mainChar]||0):0;
    drawDoll(cx,cy,sex,dir,frm,c.color);
    if(isMain){
      ctx.save();
      ctx.strokeStyle='#ffffff'; ctx.lineWidth=2;
      ctx.strokeRect(cx-28,cy-36,56,72);
      ctx.restore();
    }
  }

  // Flashing approach arrow toward next speaker
  if(phase==='wait'&&arrowVisible&&curLine){
    var spkr=chars[curLine.speaker];
    var mc=chars[mainChar];
    var adjacent=mc&&Math.max(wrappedDist(mc.x,spkr.x,GW),wrappedDist(mc.y,spkr.y,GH))<=1;
    if(spkr&&spkr.onStage&&!adjacent){
      var at=findArrowTile(spkr);
      if(at) drawArrow(at.x,at.y,at.dx,at.dy);
    }
  }
  // Ghastly cloud overlay
  if(cloudA>0&&cloudImg.complete&&cloudImg.naturalWidth>0){
    var R=150,r=50;
    var ox=TILE*GW/2+R*Math.cos(cloudAng)+r*Math.cos(cloudEpi);
    var oy=TILE*GH/2+(R*Math.sin(cloudAng)+r*Math.sin(cloudEpi))*0.6;
    var dw=canvas.width+2*(R+r), dh=canvas.height+2*(R+r)*0.6;
    ctx.save(); ctx.globalAlpha=cloudA;
    ctx.drawImage(cloudImg,ox-dw/2,oy-dh/2,dw,dh);
    ctx.restore();
  }
}

function stopArrow(){
  if(arrowTimer){clearInterval(arrowTimer);arrowTimer=null;}
  arrowVisible=true;
}

function wrappedDist(a,b,size){var d=Math.abs(a-b);return Math.min(d,size-d);}

function findArrowTile(spkr){
  var m=chars[mainChar];
  var dirs=[[0,-1],[0,1],[-1,0],[1,0]];
  var best=null,bestDist=Infinity;
  for(var i=0;i<dirs.length;i++){
    var tx=(spkr.x+dirs[i][0]+GW)%GW, ty=(spkr.y+dirs[i][1]+GH)%GH;
    if(walls[tx+','+ty]) continue;
    var occ=false,ks=Object.keys(chars);
    for(var j=0;j<ks.length;j++){
      var oc=chars[ks[j]];
      if(oc.onStage&&oc.x===tx&&oc.y===ty){occ=true;break;}
    }
    if(occ) continue;
    var d=m?(wrappedDist(m.x,tx,GW)+wrappedDist(m.y,ty,GH)):0;
    if(d<bestDist){bestDist=d;best={x:tx,y:ty,dx:-dirs[i][0],dy:-dirs[i][1]};}
  }
  return best;
}

function drawArrow(tx,ty,dx,dy){
  var cx=tx*TILE+TILE/2, cy=ty*TILE+TILE/2;
  var tip=26, hw=16, tail=20;
  ctx.save();
  ctx.fillStyle='#ffa657';
  ctx.globalAlpha=0.92;
  ctx.translate(cx,cy);
  ctx.rotate(Math.atan2(dy,dx));
  ctx.beginPath();
  ctx.moveTo(tip,0);
  ctx.lineTo(0,-hw);
  ctx.lineTo(0,-hw*0.4);
  ctx.lineTo(-tail,-hw*0.4);
  ctx.lineTo(-tail,hw*0.4);
  ctx.lineTo(0,hw*0.4);
  ctx.lineTo(0,hw);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawDoll(cx,cy,sex,dir,frame,outfit){
  var x=cx-12, y=cy-16;  // 24x32 sprite top-left (rendered at 2x scale)
  var m=(sex==='m');
  var la=(frame===1?2:frame===2?-2:0), ra=-la;  // leg swing
  var aa=(frame===1?2:frame===2?-2:0), ab=-aa;  // arm swing

  ctx.save();
  ctx.translate(cx,cy); ctx.scale(2,2); ctx.translate(-cx,-cy);
  if(dir==='right'){ctx.translate(cx,0);ctx.scale(-1,1);ctx.translate(-cx,0);}
  var side=(dir==='left'||dir==='right');
  var back=(dir==='up');

  // ── Legs / skirt ──────────────────────────────────────────────
  if(m){
    ctx.fillStyle=PANT;
    if(side){
      ctx.fillRect(x+7,y+21+la,5,10); ctx.fillRect(x+13,y+21+ra,5,10);
    } else {
      ctx.fillRect(x+4,y+21+la,6,10); ctx.fillRect(x+14,y+21+ra,6,10);
    }
    ctx.fillStyle=SHOE;
    if(side){
      ctx.fillRect(x+6,y+30+la,7,2); ctx.fillRect(x+12,y+30+ra,7,2);
    } else {
      ctx.fillRect(x+3,y+30+la,8,2); ctx.fillRect(x+13,y+30+ra,8,2);
    }
  } else {
    ctx.fillStyle=outfit;
    ctx.beginPath();
    ctx.moveTo(x+5,y+21); ctx.lineTo(x+19,y+21);
    ctx.lineTo(x+22,y+32); ctx.lineTo(x+2,y+32);
    ctx.closePath(); ctx.fill();
    ctx.fillStyle=SHOE;
    ctx.fillRect(x+4,y+31,6,1); ctx.fillRect(x+14,y+31,6,1);
  }

  // ── Torso ──────────────────────────────────────────────────────
  ctx.fillStyle=outfit;
  if(side){
    ctx.fillRect(x+5,y+11,13,10);
  } else {
    ctx.fillRect(x+4,y+11,16,10);
  }
  ctx.fillStyle=BELT;
  if(side){ ctx.fillRect(x+5,y+20,13,2); } else { ctx.fillRect(x+4,y+20,16,2); }

  // ── Arms ──────────────────────────────────────────────────────
  ctx.fillStyle=outfit;
  if(side){
    ctx.fillRect(x+14,y+11+aa,3,9); ctx.fillRect(x+7,y+11+ab,3,9);
    ctx.fillStyle=SKIN;
    ctx.fillRect(x+14,y+19+aa,3,3); ctx.fillRect(x+7,y+19+ab,3,3);
  } else {
    ctx.fillRect(x+1,y+11+aa,3,9); ctx.fillRect(x+20,y+11+ab,3,9);
    ctx.fillStyle=SKIN;
    ctx.fillRect(x+1,y+19+aa,3,3); ctx.fillRect(x+20,y+19+ab,3,3);
  }

  // ── Head ──────────────────────────────────────────────────────
  ctx.fillStyle=SKIN;
  ctx.beginPath(); ctx.arc(x+12,y+5,5,0,Math.PI*2); ctx.fill();

  // ── Hair ──────────────────────────────────────────────────────
  ctx.fillStyle=m?HAIR_M:HAIR_F;
  ctx.fillRect(x+7,y+0,10,3);
  ctx.fillRect(x+7,y+0,2,7); ctx.fillRect(x+15,y+0,2,7);
  if(!m){ ctx.fillRect(x+5,y+0,2,13); ctx.fillRect(x+17,y+0,2,13); }
  if(side&&!m){ ctx.fillRect(x+17,y+0,2,13); }

  // ── Face ──────────────────────────────────────────────────────
  if(!back){
    ctx.fillStyle='#1a0a00';
    if(side){
      ctx.fillRect(x+15,y+5,2,2);
    } else {
      ctx.fillRect(x+9,y+5,2,2); ctx.fillRect(x+13,y+5,2,2);
    }
  }

  ctx.restore();
}

document.addEventListener('keydown',function(e){
  var dirs={ArrowUp:[0,-1],ArrowDown:[0,1],ArrowLeft:[-1,0],ArrowRight:[1,0]};
  if(dirs[e.key]){e.preventDefault();move(dirs[e.key][0],dirs[e.key][1]);}
});

function tickClouds(){
  cloudTick++;
  if(cloudPhaseC==='wait'){
    if(cloudTick>300){cloudPhaseC='in';cloudTick=0;}
  }else if(cloudPhaseC==='in'){
    cloudAng+=0.006;cloudEpi+=0.017;
    cloudA=Math.min(0.5,cloudA+0.008);
    if(cloudA>=0.5){cloudPhaseC='roam';cloudTick=0;}
  }else if(cloudPhaseC==='roam'){
    cloudAng+=0.006;cloudEpi+=0.017;
    if(cloudTick>350){cloudPhaseC='out';cloudTick=0;}
  }else{
    cloudAng+=0.006;cloudEpi+=0.017;
    cloudA=Math.max(0,cloudA-0.008);
    if(cloudA<=0){cloudPhaseC='wait';cloudTick=0;}
  }
  render();
}
setInterval(tickClouds,50);

loadScene(SCENE_DATA);
</script>
</body></html>"
            .Replace("CLOUD_DATA", _cloudPng!)
            .Replace("SCENE_DATA", firstSceneJson);
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
