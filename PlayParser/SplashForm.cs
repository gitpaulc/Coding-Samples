
using System.Drawing;
using System.Drawing.Imaging;
using System.Drawing.Drawing2D;
using System.Drawing.Text;

namespace PlayParser
{
    public class SplashForm : Form
    {
        private readonly System.Windows.Forms.Timer _lingerTimer = new() { Interval = 5000 };
        private readonly Bitmap    _shakespeareBmp;
        private readonly Rectangle _imageRect;
        private bool  _canDismiss = false;
        private readonly Panel _canvas;

        public SplashForm()
        {
            FormBorderStyle = FormBorderStyle.None;
            StartPosition   = FormStartPosition.CenterScreen;
            ClientSize      = new Size(900, 540);
            BackColor       = Color.Black;
            TopMost         = true;

            string shakePath    = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "shakespeare.png");
            _shakespeareBmp     = File.Exists(shakePath) ? new Bitmap(shakePath) : MakeSolid(Color.DimGray);
            _imageRect          = CalcZoomRect(_shakespeareBmp.Size, new Rectangle(0, 0, 900, 540));

            _canvas = new DoubleBufferedPanel { Dock = DockStyle.Fill, BackColor = Color.Black };
            _canvas.Paint += (_, e) => DrawFrame(e.Graphics);
            _canvas.Click += (_, _) => { if (_canDismiss) Close(); };
            Controls.Add(_canvas);

            _lingerTimer.Tick += (_, _) =>
            {
                _lingerTimer.Stop();
                _canDismiss = true;
                _canvas.Invalidate();
            };
            _lingerTimer.Start();
        }

        private static Rectangle CalcZoomRect(Size imgSize, Rectangle panel)
        {
            double imgAR   = (double)imgSize.Width / imgSize.Height;
            double panelAR = (double)panel.Width   / panel.Height;
            int w, h, x, y;
            if (imgAR > panelAR)
            {
                w = panel.Width;
                h = (int)(panel.Width / imgAR);
                x = 0;
                y = (panel.Height - h) / 2;
            }
            else
            {
                h = panel.Height;
                w = (int)(panel.Height * imgAR);
                x = (panel.Width - w) / 2;
                y = 0;
            }
            return new Rectangle(x, y, w, h);
        }

        private void DrawFrame(Graphics g)
        {
            g.InterpolationMode  = InterpolationMode.HighQualityBicubic;
            g.CompositingQuality = CompositingQuality.HighQuality;

            g.DrawImage(_shakespeareBmp, _imageRect,
                        0, 0, _shakespeareBmp.Width, _shakespeareBmp.Height, GraphicsUnit.Pixel);

            if (_canDismiss)
                DrawText(g);
        }

        private static void DrawText(Graphics g)
        {
            g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;

            using var titleFont    = new Font("Segoe UI", 56, FontStyle.Bold,    GraphicsUnit.Pixel);
            using var subtitleFont = new Font("Segoe UI", 24, FontStyle.Regular, GraphicsUnit.Pixel);
            using var hintFont     = new Font("Segoe UI", 16, FontStyle.Italic,  GraphicsUnit.Pixel);
            using var whiteBr      = new SolidBrush(Color.White);

            g.DrawString("Play Parser",            titleFont,    whiteBr, 405, 162);
            g.DrawString("Theatrical Viewing App", subtitleFont, whiteBr, 408, 237);
            g.DrawString("Paul Cernea",            subtitleFont, whiteBr, 408, 270);
            g.DrawString("Click to continue",      hintFont,     whiteBr, 408, 405);
        }

        private static Bitmap MakeSolid(Color c)
        {
            var b = new Bitmap(1, 1);
            b.SetPixel(0, 0, c);
            return b;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _lingerTimer.Dispose();
                _shakespeareBmp.Dispose();
            }
            base.Dispose(disposing);
        }
    }

    internal sealed class DoubleBufferedPanel : Panel
    {
        public DoubleBufferedPanel() { DoubleBuffered = true; }
    }
}
