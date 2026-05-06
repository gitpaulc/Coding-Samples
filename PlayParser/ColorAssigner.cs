
namespace PlayParser
{
    public static class ColorAssigner
    {
        private static readonly (int R, int G, int B)[] Palette =
        {
            (255,  80,  80),   // red
            (255, 150,  50),   // orange
            (255, 220,  50),   // yellow
            (180, 240,  50),   // yellow-green
            ( 70, 230,  70),   // green
            ( 50, 220, 160),   // seafoam
            ( 50, 215, 230),   // cyan
            ( 80, 175, 255),   // cornflower blue
            (120, 130, 255),   // blue-violet
            (190,  90, 255),   // violet
            (245,  80, 210),   // magenta
            (255,  80, 140),   // hot pink
            (255, 140, 120),   // salmon
            (255, 190, 100),   // peach
            (230, 255, 100),   // lime
            (120, 255, 170),   // mint
            (120, 230, 255),   // sky
            (170, 170, 255),   // lavender
            (220, 150, 255),   // lilac
            (255, 160, 210),   // pink
            (255, 225, 130),   // cream
            (180, 255, 180),   // light green
            (170, 210, 255),   // pale blue
            (235, 200, 255),   // pale purple
        };

        // Perceptual squared distance between two RGB colors.
        private static double PerceivedDist2((int R, int G, int B) a, (int R, int G, int B) b)
        {
            double dr = a.R - b.R, dg = a.G - b.G, db = a.B - b.B;
            return 0.299 * dr * dr + 0.587 * dg * dg + 0.114 * db * db;
        }

        // Assigns colors to all actors in the play so that actors sharing any scene
        // receive maximally distinct colors. Colors are stable (sorted-degree order)
        // and all palette entries are bright enough to read on a black background.
        public static Dictionary<string, string> BuildColorMap(Play play)
        {
            // Build co-presence adjacency graph.
            var adj = new Dictionary<string, HashSet<string>>(StringComparer.Ordinal);
            foreach (var actor in play.Actors)
                adj[actor] = new HashSet<string>(StringComparer.Ordinal);

            foreach (var kvp in play.GetActorsInEachScene())
            {
                var inScene = kvp.Value;
                for (int i = 0; i < inScene.Count; i++)
                    for (int j = i + 1; j < inScene.Count; j++)
                    {
                        if (!adj.ContainsKey(inScene[i]) || !adj.ContainsKey(inScene[j]))
                            continue;
                        adj[inScene[i]].Add(inScene[j]);
                        adj[inScene[j]].Add(inScene[i]);
                    }
            }

            // Process actors from most-connected to least-connected.
            var actors = new List<string>(play.Actors);
            actors.Sort((a, b) => adj[b].Count.CompareTo(adj[a].Count));

            var assigned = new Dictionary<string, int>(StringComparer.Ordinal);
            var map = new Dictionary<string, string>(StringComparer.Ordinal);

            foreach (var actor in actors)
            {
                // Collect palette indices used by direct scene-neighbors.
                var refIndices = new List<int>();
                foreach (var nb in adj[actor])
                    if (assigned.TryGetValue(nb, out int ni))
                        refIndices.Add(ni);

                // If no neighbors have been assigned yet, use all assigned colors so
                // isolated actors still spread across the palette.
                if (refIndices.Count == 0)
                    refIndices.AddRange(assigned.Values);

                // Pick the palette entry with the maximum minimum perceptual distance
                // to the reference colors.
                int bestIdx = 0;
                double bestScore = -1;
                for (int ci = 0; ci < Palette.Length; ci++)
                {
                    double minDist = double.MaxValue;
                    foreach (int ri in refIndices)
                        minDist = Math.Min(minDist, PerceivedDist2(Palette[ci], Palette[ri]));
                    if (minDist > bestScore)
                    {
                        bestScore = minDist;
                        bestIdx = ci;
                    }
                }

                assigned[actor] = bestIdx;
                var (r, g, b) = Palette[bestIdx];
                map[actor] = $"#{r:x2}{g:x2}{b:x2}";
            }

            return map;
        }
    }
}
