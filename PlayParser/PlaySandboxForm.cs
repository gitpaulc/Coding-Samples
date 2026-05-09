using System.Runtime.InteropServices;
using System.Text;

namespace PlayParser
{
    public class PlaySandboxForm : Form
    {
        [DllImport("user32.dll")] private static extern bool SendMessage(IntPtr h, int msg, bool w, int l);
        private const int WmSetRedraw = 11;
        private ToolStrip _toolStrip = null!;
        private ToolStripComboBox _eraCombo = null!;
        private ToolStripComboBox _playCombo = null!;
        private ToolStripButton _btnCreate = null!;
        private ToolStripButton _btnReset  = null!;
        private RichTextBox _sourceBox = null!;
        private StatusStrip _statusStrip = null!;
        private ToolStripStatusLabel _statusLabel = null!;
        private System.Windows.Forms.Timer _saveTimer = null!;
        private PlayParserForm? _parserForm;

        private bool _loading;
        private bool _dirty;
        private string[]? _originalLines;

        private static readonly Color TextDefault = Color.Black;
        private static readonly Color TextEdited  = Color.FromArgb(0, 160, 50);

        // ── Dark theme palette (matches PlayParserForm / RPG bar) ─────────────
        private static readonly Color BgDark    = Color.FromArgb(13,  17,  23);
        private static readonly Color BgStrip   = Color.FromArgb(22,  27,  34);
        private static readonly Color BgButton  = Color.FromArgb(33,  38,  45);
        private static readonly Color BgHover   = Color.FromArgb(48,  54,  61);
        private static readonly Color BgPress   = Color.FromArgb(64,  72,  80);
        private static readonly Color FgPrimary = Color.FromArgb(201, 209, 217);
        private static readonly Color FgMuted   = Color.FromArgb(139, 148, 158);
        private static readonly Color FgBorder  = Color.FromArgb(64,  72,  80);

        public PlaySandboxForm()
        {
            Text = "Play Sandbox";
            Size = new Size(1000, 750);
            MinimumSize = new Size(600, 400);
            StartPosition = FormStartPosition.CenterScreen;
            BackColor = BgDark;

            try
            {
                var icoPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "shakespeare.ico");
                if (File.Exists(icoPath)) Icon = new Icon(icoPath);
            }
            catch { }

            BuildToolStrip();
            BuildStatusStrip();
            BuildSourceBox();

            _saveTimer = new System.Windows.Forms.Timer { Interval = 600 };
            _saveTimer.Tick += (_, _) => { _saveTimer.Stop(); SaveEdited(); };

            // Populate era → triggers play combo → triggers load
            _eraCombo.Items.Add("Classical");
            _eraCombo.Items.Add("Renaissance");
            _eraCombo.SelectedIndex = 1;
        }

        private void BuildToolStrip()
        {
            _toolStrip = new ToolStrip
            {
                GripStyle  = ToolStripGripStyle.Hidden,
                Padding    = new Padding(6, 3, 6, 3),
                BackColor  = BgStrip,
                Renderer   = new DarkStripRenderer()
            };

            // ── Era ───────────────────────────────────────────────────────────
            var lblEra = new ToolStripLabel("Era:")
            {
                ForeColor = FgMuted,
                Font      = new Font("Segoe UI", 8.5f)
            };
            _toolStrip.Items.Add(lblEra);

            _eraCombo = new ToolStripComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                AutoSize      = false,
                Width         = 120
            };
            StyleCombo(_eraCombo);
            _eraCombo.SelectedIndexChanged += EraCombo_Changed;
            _toolStrip.Items.Add(_eraCombo);

            _toolStrip.Items.Add(new ToolStripSeparator());

            // ── Play ──────────────────────────────────────────────────────────
            var lblPlay = new ToolStripLabel("Play:")
            {
                ForeColor = FgMuted,
                Font      = new Font("Segoe UI", 8.5f)
            };
            _toolStrip.Items.Add(lblPlay);

            _playCombo = new ToolStripComboBox
            {
                DropDownStyle = ComboBoxStyle.DropDownList,
                AutoSize      = false,
                Width         = 210
            };
            StyleCombo(_playCombo);
            _playCombo.SelectedIndexChanged += PlayCombo_Changed;
            _toolStrip.Items.Add(_playCombo);

            _toolStrip.Items.Add(new ToolStripSeparator());

            // ── Create ────────────────────────────────────────────────────────
            _btnCreate = new ToolStripButton("▶ Create")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                ForeColor    = FgPrimary,
                Font         = new Font("Segoe UI", 8.5f),
                ToolTipText  = "Split source and generate play report"
            };
            _btnCreate.Click += BtnCreate_Click;
            _toolStrip.Items.Add(_btnCreate);

            _toolStrip.Items.Add(new ToolStripSeparator());

            // ── Reset ─────────────────────────────────────────────────────────
            _btnReset = new ToolStripButton("↺ Reset")
            {
                DisplayStyle = ToolStripItemDisplayStyle.Text,
                ForeColor    = FgMuted,
                Font         = new Font("Segoe UI", 8.5f),
                ToolTipText  = "Revert to original source from Sources folder",
                Visible      = false
            };
            _btnReset.Click += BtnReset_Click;
            _toolStrip.Items.Add(_btnReset);

            Controls.Add(_toolStrip);
        }

        private static void StyleCombo(ToolStripComboBox combo)
        {
            combo.Font = new Font("Segoe UI", 8.5f);
            if (combo.ComboBox != null)
            {
                combo.ComboBox.BackColor = Color.FromArgb(33, 38, 45);
                combo.ComboBox.ForeColor = Color.FromArgb(201, 209, 217);
                combo.ComboBox.FlatStyle = FlatStyle.Flat;
            }
        }

        private void BuildStatusStrip()
        {
            _statusStrip = new StatusStrip
            {
                BackColor  = BgStrip,
                ForeColor  = FgMuted,
                SizingGrip = false,
                Renderer   = new DarkStripRenderer()
            };
            _statusLabel = new ToolStripStatusLabel("Ready.")
            {
                TextAlign = ContentAlignment.MiddleLeft,
                ForeColor = FgMuted,
                Font      = new Font("Segoe UI", 8.5f)
            };
            _statusStrip.Items.Add(_statusLabel);
            Controls.Add(_statusStrip);
        }

        private void BuildSourceBox()
        {
            _sourceBox = new RichTextBox
            {
                Dock        = DockStyle.Fill,
                Font        = new Font("Consolas", 11f),
                BackColor   = Color.White,
                ForeColor   = Color.Black,
                BorderStyle = BorderStyle.None,
                WordWrap    = false,
                ScrollBars  = RichTextBoxScrollBars.Both,
                AcceptsTab  = true
            };
            _sourceBox.TextChanged += SourceBox_TextChanged;
            Controls.Add(_sourceBox);
        }

        // ── Combo logic ───────────────────────────────────────────────────────

        private void EraCombo_Changed(object? sender, EventArgs e)
        {
            string era = _eraCombo.SelectedItem as string ?? "Renaissance";
            _playCombo.Items.Clear();

            var sources = GutenbergSplitter.GetSourcesFolder();
            if (!Directory.Exists(sources)) return;

            foreach (var file in Directory.GetFiles(sources, "*.txt").OrderBy(f => f))
            {
                string name = Path.GetFileNameWithoutExtension(file);
                if (Play.GetPlayEra(name) == era)
                    _playCombo.Items.Add(name);
            }

            if (_playCombo.Items.Count > 0)
                _playCombo.SelectedIndex = 0;
        }

        private void PlayCombo_Changed(object? sender, EventArgs e)
        {
            string? playName = _playCombo.SelectedItem as string;
            if (playName != null) LoadSourceText(playName);
        }

        // ── Source text load / save ───────────────────────────────────────────

        private void LoadSourceText(string playName)
        {
            _saveTimer.Stop();
            if (_dirty) SaveEdited();

            _loading = true;
            try
            {
                // Load original for diff comparison (always from Sources/, not SourcesEdited/)
                var origPath = Path.Combine(GutenbergSplitter.GetSourcesFolder(), playName + ".txt");
                _originalLines = File.Exists(origPath)
                    ? File.ReadAllLines(origPath, new UTF8Encoding(false))
                    : null;

                string? path = ResolveSourcePath(playName);
                if (path != null)
                {
                    _sourceBox.Text = File.ReadAllText(path, new UTF8Encoding(false));
                    bool isEdited = path.Contains("SourcesEdited", StringComparison.OrdinalIgnoreCase);
                    _statusLabel.Text = isEdited ? $"Loaded (edited): {playName}" : $"Loaded: {playName}";
                }
                else
                {
                    _sourceBox.Text = "";
                    _statusLabel.Text = $"Source not found: {playName}";
                }
                _dirty = false;
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Load error: {ex.Message}";
            }
            finally
            {
                _loading = false;
            }
            ApplyDiffColoring();
            UpdateResetVisibility();
        }

        private void ApplyDiffColoring()
        {
            var lines = _sourceBox.Lines;
            int savedStart = _sourceBox.SelectionStart;
            for (int i = 0; i < lines.Length; i++)
            {
                bool diff = _originalLines == null
                    || i >= _originalLines.Length
                    || lines[i] != _originalLines[i];
                int start = _sourceBox.GetFirstCharIndexFromLine(i);
                if (start < 0) continue;
                _sourceBox.Select(start, lines[i].Length);
                _sourceBox.SelectionColor = diff ? TextEdited : TextDefault;
            }
            _sourceBox.Select(savedStart, 0);
        }

        private void ColorCurrentLine()
        {
            if (_originalLines == null) return;
            int li = _sourceBox.GetLineFromCharIndex(_sourceBox.SelectionStart);
            var lines = _sourceBox.Lines;
            if (li < 0 || li >= lines.Length) return;
            bool diff = li >= _originalLines.Length || lines[li] != _originalLines[li];
            int savedStart = _sourceBox.SelectionStart;
            int start = _sourceBox.GetFirstCharIndexFromLine(li);
            if (start >= 0)
            {
                _sourceBox.Select(start, lines[li].Length);
                _sourceBox.SelectionColor = diff ? TextEdited : TextDefault;
                _sourceBox.Select(savedStart, 0);
            }
        }

        private static string? ResolveSourcePath(string playName)
        {
            var editedPath = Path.Combine(GutenbergSplitter.GetSourcesEditedFolder(), playName + ".txt");
            if (File.Exists(editedPath)) return editedPath;

            var srcPath = Path.Combine(GutenbergSplitter.GetSourcesFolder(), playName + ".txt");
            if (File.Exists(srcPath)) return srcPath;

            return null;
        }

        private void SourceBox_TextChanged(object? sender, EventArgs e)
        {
            if (_loading) return;
            _dirty = true;
            _saveTimer.Stop();
            _saveTimer.Start();
            _statusLabel.Text = "Modified (auto-saving)…";
            ColorCurrentLine();
        }

        private void SaveEdited()
        {
            string? playName = _playCombo.SelectedItem as string;
            if (playName == null || !_dirty) return;

            var editedDir = GutenbergSplitter.GetSourcesEditedFolder();
            Directory.CreateDirectory(editedDir);
            var path = Path.Combine(editedDir, playName + ".txt");
            try
            {
                File.WriteAllText(path, _sourceBox.Text, new UTF8Encoding(false));
                _dirty = false;
                _statusLabel.Text = $"Saved: {playName}";
                UpdateResetVisibility();
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Save error: {ex.Message}";
            }
        }

        // ── Create button ─────────────────────────────────────────────────────

        private async void BtnCreate_Click(object? sender, EventArgs e)
        {
            string? playName = _playCombo.SelectedItem as string;
            if (playName == null) return;

            _saveTimer.Stop();
            if (_dirty) SaveEdited();

            _btnCreate.Enabled = false;

            if (_parserForm == null || _parserForm.IsDisposed)
            {
                _parserForm = new PlayParserForm();
                Program.ppf = _parserForm;
                _parserForm.Show(this);
            }
            else
            {
                _parserForm.BringToFront();
            }

            _parserForm.ClearLog();

            var prevOut = Console.Out;
            Console.SetOut(new GuiTextWriter(_parserForm.AppendLog));

            List<Play> plays = new();
            try
            {
                _statusLabel.Text = "Splitting…";
                string editedFile = Path.Combine(GutenbergSplitter.GetSourcesEditedFolder(), playName + ".txt");
                string sourcesFolder = File.Exists(editedFile)
                    ? GutenbergSplitter.GetSourcesEditedFolder()
                    : GutenbergSplitter.GetSourcesFolder();

                await Task.Run(() => GutenbergSplitter.Split(playName, sourcesFolder));

                _statusLabel.Text = "Processing…";
                plays = await Task.Run(() => Program.RunSinglePlay(playName));
                _statusLabel.Text = $"Done: {playName}";
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
                _statusLabel.Text = $"Error: {ex.Message}";
            }
            finally
            {
                Console.SetOut(prevOut);
                _btnCreate.Enabled = true;
            }

            _parserForm.LoadPlays(plays);
        }

        // ── Reset button ──────────────────────────────────────────────────────

        private void UpdateResetVisibility()
        {
            string? playName = _playCombo.SelectedItem as string;
            if (playName == null) { _btnReset.Visible = false; return; }
            var editedPath = Path.Combine(GutenbergSplitter.GetSourcesEditedFolder(), playName + ".txt");
            _btnReset.Visible = File.Exists(editedPath);
        }

        private void BtnReset_Click(object? sender, EventArgs e)
        {
            string? playName = _playCombo.SelectedItem as string;
            if (playName == null) return;

            var result = MessageBox.Show(
                $"Revert \"{playName}\" to the original source? Your edits will be permanently deleted.",
                "Reset Play",
                MessageBoxButtons.OKCancel,
                MessageBoxIcon.Warning,
                MessageBoxDefaultButton.Button2);

            if (result != DialogResult.OK) return;

            _saveTimer.Stop();
            _dirty = false;

            var editedPath = Path.Combine(GutenbergSplitter.GetSourcesEditedFolder(), playName + ".txt");
            try
            {
                if (File.Exists(editedPath)) File.Delete(editedPath);
            }
            catch (Exception ex)
            {
                _statusLabel.Text = $"Reset error: {ex.Message}";
                return;
            }

            LoadSourceText(playName);
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            _saveTimer.Stop();
            if (_dirty) SaveEdited();
            base.OnFormClosing(e);
        }

        // ── Dark renderer ─────────────────────────────────────────────────────

        private sealed class DarkStripRenderer : ToolStripProfessionalRenderer
        {
            public DarkStripRenderer() : base(new DarkColorTable()) { }

            protected override void OnRenderToolStripBorder(ToolStripRenderEventArgs e)
            {
                // suppress the default light border at the bottom of the strip
                using var pen = new Pen(Color.FromArgb(48, 54, 61));
                e.Graphics.DrawLine(pen, 0, e.ToolStrip.Height - 1,
                                         e.ToolStrip.Width, e.ToolStrip.Height - 1);
            }

            protected override void OnRenderButtonBackground(ToolStripItemRenderEventArgs e)
            {
                var item = e.Item;
                var g    = e.Graphics;
                var rect = new Rectangle(2, 1, item.Width - 4, item.Height - 2);

                Color bg;
                if (!item.Enabled)
                    bg = Color.Transparent;
                else if (item.Pressed)
                    bg = Color.FromArgb(64, 72, 80);
                else if (item.Selected)
                    bg = Color.FromArgb(48, 54, 61);
                else
                    bg = Color.Transparent;

                if (bg != Color.Transparent)
                {
                    using var brush = new SolidBrush(bg);
                    g.FillRectangle(brush, rect);
                    using var pen = new Pen(Color.FromArgb(64, 72, 80));
                    g.DrawRectangle(pen, rect);
                }
            }

            protected override void OnRenderSeparator(ToolStripSeparatorRenderEventArgs e)
            {
                var g = e.Graphics;
                int mid = e.Item.Width / 2;
                using var pen = new Pen(Color.FromArgb(48, 54, 61));
                g.DrawLine(pen, mid, 3, mid, e.Item.Height - 3);
            }
        }

        private sealed class DarkColorTable : ProfessionalColorTable
        {
            private static Color S = Color.FromArgb(22, 27, 34);
            public override Color ToolStripGradientBegin           => S;
            public override Color ToolStripGradientMiddle          => S;
            public override Color ToolStripGradientEnd             => S;
            public override Color MenuStripGradientBegin           => S;
            public override Color MenuStripGradientEnd             => S;
            public override Color StatusStripGradientBegin         => S;
            public override Color StatusStripGradientEnd           => S;
            public override Color ToolStripBorder                  => Color.FromArgb(48, 54, 61);
            public override Color SeparatorDark                    => Color.FromArgb(48, 54, 61);
            public override Color SeparatorLight                   => Color.FromArgb(22, 27, 34);
            public override Color ButtonSelectedGradientBegin      => Color.FromArgb(48, 54, 61);
            public override Color ButtonSelectedGradientMiddle     => Color.FromArgb(48, 54, 61);
            public override Color ButtonSelectedGradientEnd        => Color.FromArgb(48, 54, 61);
            public override Color ButtonSelectedBorder             => Color.FromArgb(64, 72, 80);
            public override Color ButtonPressedGradientBegin       => Color.FromArgb(64, 72, 80);
            public override Color ButtonPressedGradientMiddle      => Color.FromArgb(64, 72, 80);
            public override Color ButtonPressedGradientEnd         => Color.FromArgb(64, 72, 80);
            public override Color ButtonPressedBorder              => Color.FromArgb(88, 96, 108);
            public override Color ToolStripDropDownBackground      => Color.FromArgb(33, 38, 45);
            public override Color ImageMarginGradientBegin         => S;
            public override Color ImageMarginGradientMiddle        => S;
            public override Color ImageMarginGradientEnd           => S;
        }
    }
}
