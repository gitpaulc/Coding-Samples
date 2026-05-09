using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;

namespace PlayParser;

// Generates procedural tile images for the RPG scene.
// Character sprite sheets are static assets in resource/castle/ and are NOT generated here.
internal static class SpriteGen
{
    private const int TileSize = 80; // must match JS TILE constant

    private static Color Hex(string h)
    {
        h = h.TrimStart('#');
        return Color.FromArgb(255,
            Convert.ToInt32(h[0..2], 16),
            Convert.ToInt32(h[2..4], 16),
            Convert.ToInt32(h[4..6], 16));
    }

    public static void EnsureAssets(string castleDir)
    {
        Directory.CreateDirectory(castleDir);
        var tiles = new (string Name, Func<Bitmap> Gen)[]
        {
            ("wall.png",  WallTile),
            ("floor.png", FloorTile),
        };
        foreach (var (name, gen) in tiles)
        {
            var path = Path.Combine(castleDir, name);
            if (File.Exists(path)) continue;
            using var bmp = gen();
            bmp.Save(path, ImageFormat.Png);
        }
    }

    private static Bitmap WallTile()
    {
        var bmp = new Bitmap(TileSize, TileSize, PixelFormat.Format32bppArgb);
        using var g = Graphics.FromImage(bmp);
        g.SmoothingMode = SmoothingMode.None;
        int T = TileSize;
        using (var b = new SolidBrush(Hex("#1c2128"))) g.FillRectangle(b, 0, 0, T,     T);
        using (var b = new SolidBrush(Hex("#2d333b"))) g.FillRectangle(b, 2, 2, T - 4, T - 4);
        using (var b = new SolidBrush(Hex("#373e47")))
        {
            g.FillRectangle(b, 2, 2, T - 4, 2);     // top highlight
            g.FillRectangle(b, 2, 2, 2,     T - 4); // left highlight
        }
        return bmp;
    }

    private static Bitmap FloorTile()
    {
        var bmp = new Bitmap(TileSize, TileSize, PixelFormat.Format32bppArgb);
        using var g = Graphics.FromImage(bmp);
        g.SmoothingMode = SmoothingMode.None;
        int T = TileSize;
        using (var b = new SolidBrush(Hex("#0d1117"))) g.FillRectangle(b, 0, 0, T, T);
        using var pen = new Pen(Hex("#161b22"), 1f);
        g.DrawLine(pen, 0,     0, T - 1, 0);     // top border
        g.DrawLine(pen, 0,     0, 0,     T - 1); // left border
        return bmp;
    }
}
