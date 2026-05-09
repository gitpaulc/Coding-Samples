
using Microsoft.Web.WebView2.WinForms;
using System.Text;

namespace PlayParser
{
    public class PlayParserForm : Form
    {
        Play? recentScenePlay = null;
        string recentScenePath = "";
        private static string? _cloudPng, _maleBase, _maleOutfit, _femaleBase, _femaleOutfit, _wallTile, _floorTile;
        private static bool _castleReady;
        private static void EnsureCastle()
        {
            if (_castleReady) return;
            var dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "resource", "castle");
            SpriteGen.EnsureAssets(dir);
            _castleReady = true;
        }
        private static string LoadAsset(string filename)
        {
            var path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "resource", "castle", filename);
            return "data:image/png;base64," + Convert.ToBase64String(File.ReadAllBytes(path));
        }

        // ── Layout constants ──────────────────────────────────────────────────
        private const int TabStripHeight    = 60;
        private const int TabBtnReportWidth = 120;
        private const int TabBtnSceneWidth  = 150;
        private const int TabBtnRpgWidth    = 80;
        private const int SceneBarHeight    = 38;
        private const int RpgBarHeight      = 42;
        private const int ConsolePanelPercent = 30;

        // ── Fields ────────────────────────────────────────────────────────────
        private ToolStrip toolStrip = null!;
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
        private Label _playNameLabel = null!;
        private ComboBox sceneCombo = null!;
        private StatusStrip statusStrip = null!;
        private ToolStripStatusLabel statusLabel = null!;
        private bool webViewReady = false;
        private bool sceneWebViewReady = false;
        private int selectedTab = 0;
        private List<Play> lastPlays = new();
        private string _currentPlayName = "";
        private List<Play>? _pendingPlays;
        private Dictionary<string, string>? _actorColors;
        // ── RPG tab fields ────────────────────────────────────────────────────
        private Label    tabBtnRpg    = null!;
        private Panel    rpgBar       = null!;
        private ComboBox rpgPlayCombo = null!;
        private Button   btnRpgPrev     = null!;
        private Button   btnRpgNext     = null!;
        private Button   btnRpgRestart  = null!;
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

            btnSavePdf = new ToolStripButton("Save PDF")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                Enabled = false
            };
            btnSavePdf.Click += BtnSavePdf_Click;

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

            const int RowTop = 7;
            const int RowLblTop = 11;

            // ── Play name label ───────────────────────────────────────────────
            _playNameLabel = new Label
            {
                Text = "", AutoSize = false, Width = 200, Height = 20,
                Top = RowLblTop, Left = 8,
                ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f, FontStyle.Bold),
                BackColor = Color.Transparent
            };

            // ── Scene ─────────────────────────────────────────────────────────
            var lblScene = new Label
            {
                Text = "Scene:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = RowLblTop, Left = 216
            };
            sceneCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 160, Top = RowTop, Left = 262,
                Font = new Font("Segoe UI", 8.5f)
            };

            // ── Actor ─────────────────────────────────────────────────────────
            var lblActor = new Label
            {
                Text = "Actor:", AutoSize = true,
                ForeColor = Color.FromArgb(139, 148, 158),
                Font = new Font("Segoe UI", 8.5f),
                Top = RowLblTop, Left = 434
            };
            actorCombo = new ComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                Width = 150, Top = RowTop, Left = 478,
                Font = new Font("Segoe UI", 8.5f)
            };
            actorCombo.Items.Add("None");
            actorCombo.SelectedIndex = 0;

            colorSwatch = new Panel
            {
                Width = 16, Height = 16,
                Left = 634, Top = (SceneBarHeight - 16) / 2,
                BackColor = Color.FromArgb(48, 54, 61),
                Visible = false
            };
            actorInfoBtn = new Button
            {
                Text = "Actor Info",
                Size = new Size(82, 22),
                Left = 658, Top = (SceneBarHeight - 22) / 2,
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(33, 38, 45),
                ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f),
                Cursor = Cursors.Hand,
                Visible = false
            };
            actorInfoBtn.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            actorInfoBtn.Click += ActorInfoBtn_Click;

            sceneCombo.SelectedIndexChanged += SceneCombo_Changed;
            actorCombo.SelectedIndexChanged += ActorCombo_Changed;
            sceneBar.Controls.AddRange(new Control[] {
                _playNameLabel, lblScene, sceneCombo, lblActor, actorCombo, colorSwatch, actorInfoBtn
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

            btnRpgRestart = new Button {
                Text = "↺ Restart", Size = new Size(90, 24), Left = 502, Top = RpgTop,
                FlatStyle = FlatStyle.Flat, Visible = false,
                BackColor = Color.FromArgb(33, 38, 45), ForeColor = Color.FromArgb(201, 209, 217),
                Font = new Font("Segoe UI", 8.5f), Cursor = Cursors.Hand };
            btnRpgRestart.FlatAppearance.BorderColor = Color.FromArgb(64, 72, 80);
            btnRpgRestart.Click += RpgRestart_Click;

            rpgBar.Controls.AddRange(new Control[] { lblRpgPlay, rpgPlayCombo, btnRpgStart,
                                                     btnRpgPrev, btnRpgNext, btnRpgRestart });

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
                if (_pendingPlays != null)
                {
                    webView.NavigateToString(HtmlReport.Generate(_pendingPlays));
                    _pendingPlays = null;
                }
                else
                {
                    webView.NavigateToString(SplashHtml());
                }
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
                if (!string.IsNullOrEmpty(_currentPlayName) && sceneCombo.SelectedItem is SceneItem)
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
            "<p style='font-size:1.1rem;'>Press <strong style='color:#e6edf3;'>Create</strong> in Play Sandbox to load a play.</p>" +
            "</body></html>";

        private static string SceneViewerSplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Run the parser, then select a play and scene.</p>" +
            "</body></html>";

        // ── Console logging ───────────────────────────────────────────────────

        internal void AppendLog(string text)
        {
            if (consoleBox.InvokeRequired) { consoleBox.BeginInvoke(() => AppendLog(text)); return; }
            consoleBox.AppendText(text);
            consoleBox.ScrollToCaret();
        }

        internal void ClearLog()
        {
            if (consoleBox.InvokeRequired) { consoleBox.BeginInvoke(ClearLog); return; }
            consoleBox.Clear();
        }

        // ── Load plays (called by PlaySandboxForm after Create) ───────────────

        public void LoadPlays(List<Play> plays)
        {
            lastPlays = plays;

            if (webViewReady)
                webView.NavigateToString(HtmlReport.Generate(plays));
            else
                _pendingPlays = plays;

            string? playName = plays.Count > 0 ? plays[0].playName : null;
            if (playName != null)
            {
                _currentPlayName = playName;
                _playNameLabel.Text = playName;
                LoadCurrentPlayScenes();
            }

            PopulateRpgCombos();
            btnSavePdf.Enabled = plays.Count > 0;
            statusLabel.Text = playName != null ? $"Loaded: {playName}" : "Done.";
        }

        // ── Scene viewer combo logic ──────────────────────────────────────────

        private void LoadCurrentPlayScenes()
        {
            sceneCombo.Items.Clear();
            if (string.IsNullOrEmpty(_currentPlayName)) return;

            var play = lastPlays.FirstOrDefault(p => p.playName == _currentPlayName);
            if (play == null) return;

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
            if (string.IsNullOrEmpty(_currentPlayName) || sceneCombo.SelectedItem is not SceneItem item) return;

            recentScenePlay = lastPlays.FirstOrDefault(p => p.playName == _currentPlayName);
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
            if (string.IsNullOrEmpty(_currentPlayName)) return;
            string? actorKey = actorCombo.SelectedItem as string;
            if (actorKey == null || actorKey == "None") return;
            var play = lastPlays.FirstOrDefault(p => p.playName == _currentPlayName);
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

        private async void RpgRestart_Click(object? sender, EventArgs e)
        {
            if (!rpgWebViewReady || _rpgScenes.Count == 0) return;
            var json = RpgTileEngine.ToJson(_rpgScenes[_rpgSceneIdx]);
            await rpgWebView.ExecuteScriptAsync($"loadScene({json})");
        }

        private void UpdateRpgNavButtons()
        {
            bool active = _rpgScenes.Count > 0;
            btnRpgPrev.Visible    = active && _rpgSceneIdx > 0;
            btnRpgNext.Visible    = active && _rpgSceneIdx < _rpgScenes.Count - 1;
            btnRpgRestart.Visible = active;
        }

        private static string RpgSplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Run the parser, select a play, then press " +
            "<strong style='color:#e6edf3;'>▶ Start</strong>.</p></body></html>";

        private static string GameHtml(string firstSceneJson)
        {
            EnsureCastle();
            _cloudPng   ??= LoadAsset("cloud.png");
            _maleBase   ??= LoadAsset("male.png");
            _maleOutfit ??= LoadAsset("male_outfit.png");
            _femaleBase ??= LoadAsset("female.png");
            _femaleOutfit ??= LoadAsset("female_outfit.png");
            _wallTile   ??= LoadAsset("wall.png");
            _floorTile  ??= LoadAsset("floor.png");
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
#speaker{font-weight:700;font-size:1.2rem;letter-spacing:.07em;color:#c9d1d9;margin-bottom:4px}
#speech{font-size:1.2rem;color:#e6edf3;line-height:1.55}
#hint{margin-top:10px;font-size:1.0rem;font-style:italic;color:#8b949e}
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

var mainChar='',heroChar='',chars={},timeline=[],tlIdx=0,lineCounts={};
var curLine=null,phase='idle',monoTimer=null;

var cloudImg=new Image(); cloudImg.src='data:image/png;base64,CLOUD_DATA';
var maleBaseImg=new Image(); maleBaseImg.src='data:image/png;base64,MALE_BASE_DATA';
var maleOutfitImg=new Image(); maleOutfitImg.src='data:image/png;base64,MALE_OUTFIT_DATA';
var femaleBaseImg=new Image(); femaleBaseImg.src='data:image/png;base64,FEMALE_BASE_DATA';
var femaleOutfitImg=new Image(); femaleOutfitImg.src='data:image/png;base64,FEMALE_OUTFIT_DATA';
var wallImg=new Image(); wallImg.src='data:image/png;base64,WALL_DATA';
var floorImg=new Image(); floorImg.src='data:image/png;base64,FLOOR_DATA';
var offCanvas=document.createElement('canvas'); offCanvas.width=48; offCanvas.height=64;
var offCtx=offCanvas.getContext('2d');
var cloudA=0,cloudAng=0,cloudEpi=0,cloudPhaseC='wait',cloudTick=0;
var charSex={},charDir={},charFrame={},walkTick=0;
var arrowVisible=true,arrowTimer=null;
var walls={};

function showVictory(){
  var v=document.getElementById('victory');
  v.style.display='flex';
  document.querySelector('#victory p').textContent='All scenes complete.';
}

function loadScene(data){
  if(monoTimer){clearTimeout(monoTimer);monoTimer=null;}
  if(arrowTimer){clearInterval(arrowTimer);arrowTimer=null;} arrowVisible=true;
  mainChar=data.mainChar; heroChar=data.heroChar||''; lineCounts=data.lineCounts||{};
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

function setSpeech(spkr,txt,color){
  var el=document.getElementById('speaker');
  el.textContent=spkr;
  el.style.color=color||'#c9d1d9';
  document.getElementById('speech').textContent=txt;
}

function step(){
  while(tlIdx<timeline.length){
    var ev=timeline[tlIdx];

    if(ev.type==='enter'){
      for(var i=0;i<ev.chars.length;i++) if(chars[ev.chars[i]]) chars[ev.chars[i]].onStage=true;
      // When the scene hero enters, they become the player character.
      if(heroChar && heroChar!==mainChar && ev.chars.indexOf(heroChar)>=0){
        mainChar=heroChar;
        document.getElementById('main-lbl').textContent='You: '+mainChar;
      }
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
      setSpeech(ev.speaker.toUpperCase()+'.', ev.text, chars[ev.speaker]?chars[ev.speaker].color:null);
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
  // Floor tiles
  var floorOk=floorImg.complete&&floorImg.naturalWidth>0;
  for(var gy=0;gy<GH;gy++) for(var gx=0;gx<GW;gx++){
    if(floorOk) ctx.drawImage(floorImg,gx*TILE,gy*TILE,TILE,TILE);
    else{ ctx.fillStyle='#0d1117'; ctx.fillRect(gx*TILE,gy*TILE,TILE,TILE); }
  }

  // Walls
  var wallOk=wallImg.complete&&wallImg.naturalWidth>0;
  var wkeys=Object.keys(walls);
  for(var wi=0;wi<wkeys.length;wi++){
    var wp=wkeys[wi].split(',');
    var wrx=parseInt(wp[0])*TILE,wry=parseInt(wp[1])*TILE;
    if(wallOk) ctx.drawImage(wallImg,wrx,wry,TILE,TILE);
    else{
      ctx.fillStyle='#1c2128'; ctx.fillRect(wrx,wry,TILE,TILE);
      ctx.fillStyle='#2d333b'; ctx.fillRect(wrx+2,wry+2,TILE-4,TILE-4);
      ctx.fillStyle='#373e47'; ctx.fillRect(wrx+2,wry+2,TILE-4,2);
                               ctx.fillRect(wrx+2,wry+2,2,TILE-4);
    }
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
  var base=sex==='m'?maleBaseImg:femaleBaseImg;
  var mask=sex==='m'?maleOutfitImg:femaleOutfitImg;
  var di={down:0,left:1,right:2,up:3};
  var sx=((di[dir]||0)*3+frame)*48;
  // Step 1: outfit-colored region via mask
  offCtx.clearRect(0,0,48,64);
  offCtx.fillStyle=outfit; offCtx.fillRect(0,0,48,64);
  offCtx.globalCompositeOperation='destination-in';
  if(mask.complete&&mask.naturalWidth>0) offCtx.drawImage(mask,sx,0,48,64,0,0,48,64);
  offCtx.globalCompositeOperation='source-over';
  // Step 2: base layer (skin, hair, shoes, etc.) on top
  if(base.complete&&base.naturalWidth>0) offCtx.drawImage(base,sx,0,48,64,0,0,48,64);
  // Step 3: blit to main canvas
  ctx.drawImage(offCanvas,cx-24,cy-32);
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

// Defer first scene load until all sprite/tile images have decoded
var _scene0=SCENE_DATA;
var _imgPending=6;
function _imgDone(){ if(--_imgPending<=0) loadScene(_scene0); }
[maleBaseImg,maleOutfitImg,femaleBaseImg,femaleOutfitImg,wallImg,floorImg].forEach(function(img){
  if(img.complete&&img.naturalWidth>0) _imgDone();
  else{ img.onload=_imgDone; img.onerror=_imgDone; }
});
</script>
</body></html>"
            .Replace("CLOUD_DATA",         _cloudPng!)
            .Replace("MALE_BASE_DATA",     _maleBase!)
            .Replace("MALE_OUTFIT_DATA",   _maleOutfit!)
            .Replace("FEMALE_BASE_DATA",   _femaleBase!)
            .Replace("FEMALE_OUTFIT_DATA", _femaleOutfit!)
            .Replace("WALL_DATA",          _wallTile!)
            .Replace("FLOOR_DATA",         _floorTile!)
            .Replace("SCENE_DATA",         firstSceneJson);
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
