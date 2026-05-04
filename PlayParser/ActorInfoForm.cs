
namespace PlayParser
{
    internal sealed class ActorInfoForm : Form
    {
        private const int Pad = 22;
        private const int LabelW = 118;
        private const int ContentW = 360;

        public ActorInfoForm(Play play, string actorKey)
        {
            Text = "Actor Info";
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            StartPosition = FormStartPosition.CenterParent;
            BackColor = Color.FromArgb(13, 17, 23);
            ForeColor = Color.FromArgb(201, 209, 217);
            Font = new Font("Segoe UI", 9.5f);

            var scroll = new Panel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                BackColor = Color.FromArgb(13, 17, 23)
            };
            Controls.Add(scroll);

            int y = Pad;

            // Actor name heading
            var nameFont = new Font("Segoe UI", 14f, FontStyle.Bold);
            var nameLabel = new Label
            {
                Text = actorKey,
                Location = new Point(Pad, y),
                AutoSize = true,
                Font = nameFont,
                ForeColor = Color.FromArgb(230, 237, 243),
                BackColor = Color.Transparent
            };
            scroll.Controls.Add(nameLabel);
            y += nameLabel.PreferredHeight + 10;

            // Divider
            scroll.Controls.Add(new Panel
            {
                Location = new Point(Pad, y),
                Size = new Size(ContentW, 1),
                BackColor = Color.FromArgb(33, 38, 45)
            });
            y += 14;

            // Gender
            var gender = play.actorGenders.TryGetValue(actorKey, out var g) ? g : ActorGender.Neutral;
            AddRow(scroll, "Gender", gender.ToString(), ref y);

            // Total lines
            int totalLines = play.TotalLineCounts().TryGetValue(actorKey, out var tl) ? tl : 0;
            AddRow(scroll, "Total lines", totalLines == 0 ? "None" : totalLines.ToString(), ref y);

            // Scenes
            if (play.scenesPresent.TryGetValue(actorKey, out var scenes) && scenes.Count > 0)
            {
                AddRow(scroll, "Scenes present", scenes.Count.ToString(), ref y);
                y += 8;

                play.lineCounts.TryGetValue(actorKey, out var lc);
                foreach (var scene in scenes.OrderBy(s => SceneViewer.SceneOrder(s)))
                {
                    int n = lc != null && lc.TryGetValue(scene, out var ln) ? ln : 0;
                    AddSceneRow(scroll, SceneViewer.SceneLabelLong(scene),
                        n > 0 ? $"{n} lines" : "entrance only", ref y);
                }
            }

            y += Pad;
            ClientSize = new Size(ContentW + Pad * 2, Math.Min(Math.Max(y, 220), 580));
        }

        private static void AddRow(Panel p, string label, string value, ref int y)
        {
            p.Controls.Add(new Label
            {
                Text = label + ":",
                Location = new Point(Pad, y), Size = new Size(LabelW, 20),
                ForeColor = Color.FromArgb(139, 148, 158), BackColor = Color.Transparent
            });
            p.Controls.Add(new Label
            {
                Text = value,
                Location = new Point(Pad + LabelW + 4, y), Size = new Size(ContentW - LabelW - 4, 20),
                ForeColor = Color.FromArgb(201, 209, 217), BackColor = Color.Transparent
            });
            y += 22;
        }

        private static void AddSceneRow(Panel p, string scene, string lines, ref int y)
        {
            p.Controls.Add(new Label
            {
                Text = scene,
                Location = new Point(Pad + 16, y), Size = new Size(230, 19),
                ForeColor = Color.FromArgb(100, 110, 120), BackColor = Color.Transparent
            });
            p.Controls.Add(new Label
            {
                Text = lines,
                Location = new Point(Pad + 252, y), Size = new Size(ContentW - 252, 19),
                ForeColor = Color.FromArgb(169, 179, 189), BackColor = Color.Transparent
            });
            y += 20;
        }
    }
}
