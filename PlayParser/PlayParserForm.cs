
using Microsoft.Web.WebView2.WinForms;
using System.Text;

namespace PlayParser
{
    public class PlayParserForm : Form
    {
        private ToolStrip toolStrip = null!;
        private ToolStripButton btnRun = null!;
        private ToolStripButton btnSavePdf = null!;
        private SplitContainer split = null!;
        private RichTextBox consoleBox = null!;
        private WebView2 webView = null!;
        private StatusStrip statusStrip = null!;
        private ToolStripStatusLabel statusLabel = null!;
        private bool webViewReady = false;

        public PlayParserForm()
        {
            Text = "PlayParser";
            Size = new Size(1440, 900);
            MinimumSize = new Size(900, 600);
            StartPosition = FormStartPosition.CenterScreen;

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
            split = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Vertical
            };

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

            webView = new WebView2 { Dock = DockStyle.Fill };
            split.Panel2.Controls.Add(webView);

            Controls.Add(split);
        }

        private async Task InitWebView()
        {
            split.Panel1MinSize = 200;
            split.Panel2MinSize = 300;
            split.SplitterDistance = Math.Max(200, split.Width * 30 / 100);

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

        private static string SplashHtml() =>
            "<html><body style='font-family:\"Segoe UI\",sans-serif;color:#8b949e;background:#0d1117;" +
            "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>" +
            "<p style='font-size:1.1rem;'>Press <strong style='color:#e6edf3;'>Run</strong> to analyse the plays.</p>" +
            "</body></html>";

        private void AppendLog(string text)
        {
            if (consoleBox.InvokeRequired) { consoleBox.BeginInvoke(() => AppendLog(text)); return; }
            consoleBox.AppendText(text);
            consoleBox.ScrollToCaret();
        }

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

            string html = HtmlReport.Generate(plays);
            if (webViewReady)
                webView.NavigateToString(html);

            btnRun.Enabled = true;
            btnSavePdf.Enabled = true;
            statusLabel.Text = "Done.";
        }

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
