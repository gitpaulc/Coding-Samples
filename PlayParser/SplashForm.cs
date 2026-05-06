
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;

namespace PlayParser
{
    public class SplashForm : Form
    {
        private readonly System.Windows.Forms.Timer _timer = new() { Interval = 2000 };

        public SplashForm()
        {
            FormBorderStyle = FormBorderStyle.None;
            StartPosition   = FormStartPosition.CenterScreen;
            ClientSize      = new Size(600, 340);
            BackColor       = Color.FromArgb(13, 17, 23);
            TopMost         = true;

            string splashPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "splash.png");
            Image img = File.Exists(splashPath) ? Image.FromFile(splashPath) : MakePlaceholder();

            var pb = new PictureBox
            {
                Image    = img,
                SizeMode = PictureBoxSizeMode.Zoom,
                Dock     = DockStyle.Fill,
                BackColor = Color.FromArgb(13, 17, 23)
            };
            pb.Click += (_, _) => Close();
            Controls.Add(pb);

            _timer.Tick += (_, _) => { _timer.Stop(); Close(); };
            _timer.Start();
        }

        private Image MakePlaceholder()
        {
            const int W = 600, H = 340;
            var bmp = new Bitmap(W, H);
            using var g = Graphics.FromImage(bmp);
            g.SmoothingMode      = SmoothingMode.AntiAlias;
            g.TextRenderingHint  = TextRenderingHint.ClearTypeGridFit;
            g.Clear(Color.FromArgb(13, 17, 23));

            // Subtle gradient band across the middle
            using var grad = new LinearGradientBrush(
                new Rectangle(0, 80, W, 180),
                Color.FromArgb(30, 100, 80, 200),
                Color.FromArgb(0, 13, 17, 23),
                LinearGradientMode.Vertical);
            g.FillRectangle(grad, 0, 80, W, 180);

            // Shakespeare icon on the left
            string icoPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "shakespeare.ico");
            if (File.Exists(icoPath))
            {
                using var icon = new Icon(icoPath, 128, 128);
                using var iconBmp = icon.ToBitmap();
                g.DrawImage(iconBmp, 70, 80, 160, 160);
            }

            // Title
            using var titleFont    = new Font("Segoe UI", 38, FontStyle.Bold,    GraphicsUnit.Pixel);
            using var subtitleFont = new Font("Segoe UI", 16, FontStyle.Regular, GraphicsUnit.Pixel);
            using var hintFont     = new Font("Segoe UI",  11, FontStyle.Italic,  GraphicsUnit.Pixel);
            using var titleBr    = new SolidBrush(Color.FromArgb(201, 209, 217));
            using var subtitleBr = new SolidBrush(Color.FromArgb(139, 148, 158));
            using var hintBr     = new SolidBrush(Color.FromArgb(72, 79, 88));

            g.DrawString("Play Parser",             titleFont,    titleBr,    270, 108);
            g.DrawString("Theatrical Viewing App", subtitleFont, subtitleBr, 272, 158);
            g.DrawString("Paul Cernea",            subtitleFont, subtitleBr, 272, 184);
            g.DrawString("Click to continue",      hintFont,     hintBr,     272, 270);

            // Bottom rule
            using var rule = new Pen(Color.FromArgb(33, 38, 45), 1);
            g.DrawLine(rule, 0, H - 1, W, H - 1);

            return bmp;
        }
    }
}
