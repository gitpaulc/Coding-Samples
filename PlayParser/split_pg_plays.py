"""
Extract Hamlet, King Henry IV Part 1, and King Henry IV Part 2 from Project
Gutenberg source files, split into individual scene files, and write them to
the PlayParser ScenesIn directories, formatted to be compatible with PlayParser.

Source files:
  /tmp/hamlet_pg.txt       — PG eBook #1524 (individual Hamlet file)
  /tmp/complete_works.txt  — PG eBook #100  (Complete Works; Henry IV sections)

Formatting applied:
  • Speaker name lines (ALL-CAPS followed by period) → period stripped
  • "Enter ... ." lines → trailing period stripped
  • [_Exit ..._] / [_Exeunt ..._] → converted to plain "Exit ..." / "Exeunt ..."
  • Other [_..._] stage-direction wrappers → unwrapped
  • Trailing whitespace stripped from every line
"""

import re, os, sys

PLAYS_DIR = r"PlayParser\Plays"

# ── Roman numeral helper ───────────────────────────────────────────────────────

def roman_to_int(s):
    vals = {"I": 1, "V": 5, "X": 10, "L": 50}
    total, prev = 0, 0
    for ch in reversed(s.upper()):
        v = vals.get(ch, 0)
        total += v if v >= prev else -v
        prev = v
    return total

# ── Line-level cleaning ────────────────────────────────────────────────────────

# Matches an entire speaker-name line: ALL CAPS (letters, spaces, hyphens,
# apostrophes) ending with a period, e.g. "BARNARDO." "LORD BARDOLPH."
_SPEAKER_RE = re.compile(r"^[A-Z][A-Z '\-]*\.$")

def clean_line(line):
    s = line.rstrip()

    # 1. [_Exit Name._]  →  Exit Name
    s = re.sub(r'\[_Exit ([^_]+?)\._\]', lambda m: 'Exit ' + m.group(1).rstrip('.'), s)
    s = re.sub(r'\[_Exit\._\]',           'Exit',   s)
    s = re.sub(r'\[_Exeunt ([^_]+?)\._\]',lambda m:'Exeunt '+m.group(1).rstrip('.'), s)
    s = re.sub(r'\[_Exeunt\._\]',         'Exeunt', s)

    # 2. Any remaining [_..._] wrapper (other stage directions) → unwrap
    s = re.sub(r'\[_(.*?)_\]', r'\1', s)

    # 3. Strip trailing period from speaker name lines ("HAMLET." → "HAMLET")
    if _SPEAKER_RE.match(s):
        s = s[:-1]

    # 4. Strip trailing period from Enter lines  ("Enter Barnardo." → "Enter Barnardo")
    if s.startswith('Enter ') and s.endswith('.'):
        s = s[:-1]

    # 5. Strip any remaining leading whitespace
    s = s.lstrip()

    return s

# ── Scene splitter ─────────────────────────────────────────────────────────────

_ACT_RE   = re.compile(r'^ACT ([IVX]+)\s*$')
_SCENE_RE = re.compile(r'^SCENE ([IVX]+)\.')

def split_into_scenes(lines):
    """
    Walk through lines of a play (already stripped to just the play text).
    Return a dict  key → list-of-lines  where key is "1.1", "2.3", "Induction".
    """
    scenes    = {}          # ordered by insertion (Python 3.7+)
    cur_key   = None
    cur_lines = []
    cur_act   = None

    def flush():
        if cur_key and cur_lines:
            # Drop leading blank lines
            start = 0
            while start < len(cur_lines) and cur_lines[start].strip() == '':
                start += 1
            # Drop trailing blank lines
            end = len(cur_lines)
            while end > start and cur_lines[end - 1].strip() == '':
                end -= 1
            scenes[cur_key] = cur_lines[start:end]

    for raw in lines:
        line  = raw.rstrip()
        stripped = line.strip()   # use for header matching; keep original for content

        # ACT header
        m = _ACT_RE.match(stripped)
        if m:
            cur_act = roman_to_int(m.group(1))
            continue

        # INDUCTION  (Henry IV Part 2 — appears before ACT I)
        if stripped == 'INDUCTION':
            flush()
            cur_key   = 'Induction'
            cur_lines = []
            continue

        # SCENE header
        m = _SCENE_RE.match(stripped)
        if m and cur_act is not None:
            flush()
            scene_num = roman_to_int(m.group(1))
            cur_key   = f"{cur_act}.{scene_num}"
            cur_lines = [clean_line(stripped)]   # include the scene header itself
            continue

        # Content line
        if cur_key is not None:
            cur_lines.append(clean_line(line))

    flush()
    return scenes

# ── Writer ─────────────────────────────────────────────────────────────────────

def write_scenes(scenes, scenes_in_dir):
    os.makedirs(scenes_in_dir, exist_ok=True)
    for key, lines in scenes.items():
        path = os.path.join(scenes_in_dir, key + '.txt')
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('\n'.join(lines))
            f.write('\n')
        print(f"  wrote {key}.txt  ({len(lines)} lines)")

# ── Play extractors ────────────────────────────────────────────────────────────

def read_file(path):
    with open(path, encoding='utf-8-sig') as f:   # utf-8-sig strips BOM
        return f.readlines()

def extract_hamlet(pg_file):
    """Individual PG file #1524.  Play starts after the PG header."""
    lines = read_file(pg_file)
    # Find start: "ACT I" is the first real act header
    start = None
    for i, ln in enumerate(lines):
        if ln.strip() == 'ACT I':
            start = i
            break
    # Find end: PG footer sentinel
    end = None
    for i, ln in enumerate(lines):
        if '*** END OF THE PROJECT GUTENBERG' in ln:
            end = i
            break
    if start is None:
        sys.exit("Could not find ACT I in Hamlet file")
    play_lines = lines[start: end]
    return split_into_scenes(play_lines)

def extract_from_complete_works(cw_file, title_marker, next_title_marker):
    """
    Extract one play's section from the Complete Works file.
    title_marker      — exact text of the play's title line (stripped)
    next_title_marker — exact text of the next play's title line (to stop at)
    """
    lines = read_file(cw_file)
    start = None
    end   = len(lines)
    for i, ln in enumerate(lines):
        s = ln.strip()
        if start is None and s == title_marker:
            # The actual play text (not the table-of-contents line) starts with
            # "ACT" or "INDUCTION" shortly after the title.  Find the first ACT.
            for j in range(i, min(i + 300, len(lines))):
                if _ACT_RE.match(lines[j].strip()) or lines[j].strip() == 'INDUCTION':
                    start = j
                    break
        if start is not None and s == next_title_marker and i > start + 10:
            end = i
            break
    if start is None:
        sys.exit(f"Could not find play section: {title_marker!r}")
    play_lines = lines[start:end]
    return split_into_scenes(play_lines)

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    tmp = os.path.join(os.environ['USERPROFILE'], 'AppData', 'Local', 'Temp')
    hamlet_pg    = os.path.join(tmp, 'hamlet_pg.txt')
    complete_wks = os.path.join(tmp, 'complete_works.txt')

    config = [
        {
            'name':     'Hamlet',
            'source':   'hamlet',
        },
        {
            'name':     'King Henry IV Part 1',
            'source':   'cw',
            'title':    'THE FIRST PART OF KING HENRY THE FOURTH',
            'next':     'THE SECOND PART OF KING HENRY THE FOURTH',
        },
        {
            'name':     'King Henry IV Part 2',
            'source':   'cw',
            'title':    'THE SECOND PART OF KING HENRY THE FOURTH',
            'next':     'THE LIFE OF KING HENRY THE FIFTH',
        },
    ]

    for cfg in config:
        play_name = cfg['name']
        scenes_in = os.path.join(PLAYS_DIR, play_name, 'ScenesIn')
        print(f"\n{'='*60}")
        print(f"Processing: {play_name}")
        print(f"Output:     {scenes_in}")

        if cfg['source'] == 'hamlet':
            scenes = extract_hamlet(hamlet_pg)
        else:
            scenes = extract_from_complete_works(
                complete_wks, cfg['title'], cfg['next'])

        write_scenes(scenes, scenes_in)
        print(f"  Total scene files written: {len(scenes)}")

    print("\nDone.")

if __name__ == '__main__':
    main()
