
namespace PlayParser
{
    public enum ActorGender { Male, Female, Neutral }

    internal static class GenderDetector
    {
        private static readonly HashSet<string> MaleTitles = new(StringComparer.OrdinalIgnoreCase)
        {
            "King", "Prince", "Duke", "Lord", "Sir", "Earl", "Count", "Friar",
            "Archbishop", "Bishop", "Cardinal", "Captain", "Lieutenant", "Marshal",
            "Sergeant", "Baron", "Viscount", "Abbot"
        };

        private static readonly HashSet<string> FemaleTitles = new(StringComparer.OrdinalIgnoreCase)
        {
            "Queen", "Princess", "Duchess", "Lady", "Dame", "Countess", "Mistress",
            "Marchioness", "Abbess", "Widow"
        };

        private static readonly HashSet<string> MalePronouns = new(StringComparer.OrdinalIgnoreCase)
            { "he", "his", "him", "himself" };

        private static readonly HashSet<string> FemalePronouns = new(StringComparer.OrdinalIgnoreCase)
            { "she", "her", "hers", "herself" };

        private static readonly char[] WordSep =
            { ' ', '\t', ',', '.', '!', '?', ';', ':', '(', ')', '[', ']', '\'' };

        // Title-based detection first; pronoun-proximity fallback for untitled characters.
        public static ActorGender Detect(string actorName, IEnumerable<string> sceneFiles)
        {
            foreach (var part in actorName.Split(' '))
            {
                if (MaleTitles.Contains(part)) return ActorGender.Male;
                if (FemaleTitles.Contains(part)) return ActorGender.Female;
            }

            int male = 0, female = 0;
            string camel = Program.ToCamelCase(actorName);

            foreach (var file in sceneFiles)
            {
                var lines = Program.ReadFileAsLines(file);
                for (int i = 0; i < lines.Count; i++)
                {
                    if (!ContainsName(lines[i], camel)) continue;
                    // Score pronouns in a ±2-line window around each name mention.
                    int lo = Math.Max(0, i - 2);
                    int hi = Math.Min(lines.Count - 1, i + 2);
                    for (int j = lo; j <= hi; j++)
                        foreach (var w in lines[j].Split(WordSep, StringSplitOptions.RemoveEmptyEntries))
                        {
                            if (MalePronouns.Contains(w)) male++;
                            if (FemalePronouns.Contains(w)) female++;
                        }
                }
            }

            // Require at least 2 hits of the winning gender and a clear 2:1 ratio.
            if (male >= 2 && male >= female * 2) return ActorGender.Male;
            if (female >= 2 && female >= male * 2) return ActorGender.Female;
            return ActorGender.Neutral;
        }

        private static bool ContainsName(string line, string name)
        {
            int idx = line.IndexOf(name, StringComparison.OrdinalIgnoreCase);
            if (idx < 0) return false;
            bool leftOk  = idx == 0 || !char.IsLetter(line[idx - 1]);
            bool rightOk = idx + name.Length >= line.Length || !char.IsLetter(line[idx + name.Length]);
            return leftOk && rightOk;
        }
    }
}
