"""
split_trojan_women.py
Convert Project Gutenberg eBook #35171 (The Trojan Women, Gilbert Murray translation)
into PlayParser-compatible ScenesIn files.

Scenes (split at the * * * * * section dividers in the source):
  1.1  Prologue        – POSEIDON, PALLAS
  1.2  Ep. 1 + Ode 1   – HECUBA, CHORUS, LEADER, TALTHYBIUS, CASSANDRA
  1.3  Ep. 2 + Ode 2   – ANDROMACHE, HECUBA, CHORUS, LEADER, TALTHYBIUS
  1.4  Ep. 3 + Ode 3   – MENELAUS, HELEN, HECUBA, CHORUS
  1.5  Exodos          – HECUBA, LEADER, CHORUS, TALTHYBIUS
"""

import re, os

PLAYS_DIR = r"PlayParser\Plays"
PLAY_NAME = "The Trojan Women"
SRC_FILE  = os.path.join(os.environ['USERPROFILE'], 'AppData', 'Local', 'Temp', 'trojan_women.txt')

# Map of minor chorus-member speaker labels → CHORUS
CHORUS_MAP = {
    "FIRST WOMAN", "SECOND WOMAN", "THIRD WOMAN", "FOURTH WOMAN",
    "FIFTH WOMAN", "SIXTH WOMAN", "A WOMAN TO ANOTHER", "ANOTHER",
    "A MAIDEN", "SOME WOMEN", "OTHERS", "A WOMAN",
}

_SPEAKER_RE = re.compile(r'^\s*([A-Z][A-Z ,\'\-]+)\.\s*$')
_SUBCHOR_RE = re.compile(r'^\s*_([A-Za-z ]+)\._\s*$')
_DIVIDER_RE = re.compile(r'^\s*\*(\s+\*)+\s*$')
_STROPHE_RE = re.compile(r'^\[_?(Anti)?Strophe', re.IGNORECASE)


# ── Helpers ───────────────────────────────────────────────────────────────────

def read_lines(path):
    with open(path, encoding='utf-8-sig') as f:
        return [l.rstrip('\n') for l in f]


def join_multiline_brackets(lines):
    """
    Merge stage-direction lines that open with '[' but do not close on the
    same line.  Continuation lines are indented (start with whitespace).
    """
    out = []
    buf = None
    for line in lines:
        stripped = line.strip()
        if buf is not None:
            buf += ' ' + stripped
            # Close when the accumulated text ends with ._  or  .  or  ]
            if re.search(r'(?:\._|_\]|\.\]|\.)$', buf.rstrip()):
                out.append(buf)
                buf = None
            elif stripped == '':          # blank line always closes
                out.append(buf)
                buf = None
                out.append(line)
            continue

        if stripped.startswith('[') and not re.search(r'(?:\._|_\]|\.\]|\.)$', stripped):
            buf = stripped
            continue

        out.append(line)

    if buf:
        out.append(buf)
    return out


def strip_italics(s):
    return re.sub(r'_([^_]*)_', r'\1', s)


# ── Stage-direction converter ─────────────────────────────────────────────────

def process_bracket_dir(text):
    """Return a list of output lines for a [...]  stage direction."""
    t = text.strip()

    # Skip pure musical/structural markers
    if _STROPHE_RE.match(t):
        return []

    # ── Exit ──────────────────────────────────────────────────────────────────
    m = re.match(r'\[_?Exit_?\s+([A-Z][A-Za-z]+)', t)
    if m:
        return [f"Exit {m.group(1).title()}"]

    m = re.search(r'Exit,?\s+following_?\s+([A-Z][A-Za-z]+)', t, re.IGNORECASE)
    if m:
        return [f"Exit {m.group(1).title()}"]

    if re.search(r'ANDROMACHE', t) and re.search(r'driven off|borne.*ships', t, re.IGNORECASE):
        return ["Exit Andromache", "Exit Talthybius"]

    if re.search(r'Women go out', t, re.IGNORECASE):
        return ["Exeunt"]

    # ── Enter ─────────────────────────────────────────────────────────────────
    m = re.match(r'\[_?Enter_?\s+([A-Z][A-Za-z]+)', t)
    if m:
        return [f"Enter {m.group(1).title()}"]

    # [TALTHYBIUS, _followed by...enters...
    m = re.match(r'\[([A-Z][A-Za-z]+),?\s+', t)
    if m and re.search(r'enter', t, re.IGNORECASE):
        return [f"Enter {m.group(1).title()}"]

    # [...the King_ MENELAUS _enters...]  — use \s* so _ is optional with no gap
    m = re.search(r'(?:the King)_?\s+([A-Z][A-Za-z]+)_?\s*enters', t, re.IGNORECASE)
    if m:
        return [f"Enter {m.group(1).title()}"]

    # [_...and_ CASSANDRA _enters...] or [...NAME _enters...]
    m = re.search(r'\s+([A-Z]{3,})\s+_?\s*enters', t)
    if m:
        name = m.group(1).title()
        if name not in ('The', 'And', 'Then', 'With', 'Who'):
            return [f"Enter {name}"]

    # Chariot approaching → Andromache
    if re.search(r'chariot.*approach|chariot.*seen', t, re.IGNORECASE):
        return ["Enter Andromache"]

    # ── Plain stage direction ─────────────────────────────────────────────────
    s = re.sub(r'^\[', '', t)
    s = re.sub(r'\]$', '', s)
    s = strip_italics(s).strip()
    return [s] if s else []


def convert_line(line):
    """Convert one raw (post-join) source line → list of output strings."""
    stripped = line.strip()

    if stripped == '' or _DIVIDER_RE.match(stripped):
        return ['']

    # Sub-chorus italic label: _Others._  etc.
    if _SUBCHOR_RE.match(line):
        return ['CHORUS']

    # Speaker label
    m = _SPEAKER_RE.match(line)
    if m:
        name = m.group(1).strip()
        return ['CHORUS'] if name in CHORUS_MAP else [name]

    # Inline exit at end of dialogue: "...text [_Exit_ NAME."
    inline = re.search(r'\[_?Exit_?\s+([A-Z][A-Za-z]+)\.?\s*$', stripped)
    if inline and not stripped.startswith('['):
        before = stripped[:inline.start()].strip()
        result = []
        if before:
            result.append(strip_italics(before))
        result.append(f"Exit {inline.group(1).title()}")
        return result

    # Stage direction block
    if stripped.startswith('['):
        return process_bracket_dir(stripped)

    # Regular dialogue: strip italics + leading whitespace
    s = strip_italics(line).lstrip()
    return [s] if s else ['']


# ── Scene splitter ────────────────────────────────────────────────────────────

# Map 8 content sections (0–7) to 5 scene keys.
# After PG/intro material the play has 7 * * * * * dividers creating 8 sections.
#   §0  Prologue           → 1.1
#   §1  Ep.1 (Hecuba/Talthybius/Cassandra)   ┐
#   §2  Choral ode 1                          ┘ → 1.2
#   §3  Andromache episode  ┐
#   §4  Choral ode 2        ┘ → 1.3
#   §5  Menelaus/Helen ep.  ┐
#   §6  Choral ode 3        ┘ → 1.4
#   §7  Exodos              → 1.5
SECTION_TO_SCENE = {0:'1.1', 1:'1.2', 2:'1.2', 3:'1.3', 4:'1.3',
                    5:'1.4', 6:'1.4', 7:'1.5'}


def split_and_write(joined_lines, scenes_in_dir):
    # Find play start: first speaker-label line for POSEIDON in a heavily
    # indented position (≥ 4 spaces), which is the actual prologue, not the
    # character list ("GOD POSEIDON." has 0 indentation).
    play_start = None
    for i, l in enumerate(joined_lines):
        if re.match(r'^\s{4,}POSEIDON\.\s*$', l):
            play_start = i
            break
    if play_start is None:
        raise RuntimeError("Could not locate play start (POSEIDON speaker label)")

    # Find play end: "NOTES ON THE TROJAN WOMEN"
    play_end = len(joined_lines)
    for i, l in enumerate(joined_lines):
        if 'NOTES ON THE TROJAN WOMEN' in l:
            play_end = i
            break

    play_lines = joined_lines[play_start:play_end]

    # Split into sections on divider lines
    sections = [[]]
    for l in play_lines:
        if _DIVIDER_RE.match(l.strip()):
            sections.append([])
        else:
            sections[-1].append(l)

    print(f"  Found {len(sections)} sections (expected 8)")

    # Group sections into scenes
    scene_lines = {}
    for idx, section in enumerate(sections):
        key = SECTION_TO_SCENE.get(idx)
        if key is None:
            continue
        if key not in scene_lines:
            scene_lines[key] = []
        scene_lines[key].extend(section)

    # Process and write each scene
    os.makedirs(scenes_in_dir, exist_ok=True)
    for key in ['1.1', '1.2', '1.3', '1.4', '1.5']:
        raw   = scene_lines.get(key, [])
        output = []
        for l in raw:
            output.extend(convert_line(l))

        # Collapse multiple blank lines to one; trim edges
        collapsed, prev_blank = [], False
        for l in output:
            blank = l.strip() == ''
            if blank and prev_blank:
                continue
            collapsed.append(l)
            prev_blank = blank
        while collapsed and collapsed[0].strip()  == '': collapsed.pop(0)
        while collapsed and collapsed[-1].strip() == '': collapsed.pop()

        path = os.path.join(scenes_in_dir, key + '.txt')
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write('\n'.join(collapsed) + '\n')
        print(f"  wrote {key}.txt  ({len(collapsed)} lines)")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    raw   = read_lines(SRC_FILE)
    joined = join_multiline_brackets(raw)
    scenes_in = os.path.join(PLAYS_DIR, PLAY_NAME, 'ScenesIn')
    split_and_write(joined, scenes_in)
    print("\nDone.")

if __name__ == '__main__':
    main()
