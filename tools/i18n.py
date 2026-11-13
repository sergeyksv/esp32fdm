#!/usr/bin/env python3
"""Translations of the web interface, gettext style.

The English text in the code is the key: _("..."), N_("...") and NT_("...")
(see main/i18n.h).  Translations live in i18n/<lang>.po and can be edited with
any PO editor (Poedit).

  python3 tools/i18n.py update            extract the text, refresh every .po file
  python3 tools/i18n.py init et Eesti     start a new language
  python3 tools/i18n.py stats             how much of each language is translated
  python3 tools/i18n.py gen --out FILE    build the C tables (run by the build)

gen refuses translations that would break a page or a printf format, and turns
' into a typographic apostrophe (unless the English text has ' itself, as
markup), so a translation can never close a JavaScript string on the page.
"""

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "main"
PO_DIR = ROOT / "i18n"
POT = PO_DIR / "messages.pot"

MACRO_RE = re.compile(r'(?<![A-Za-z0-9_])(?:_|N_|NT_)\(\s*((?:"(?:[^"\\\n]|\\.)*"\s*)+)\)')
LITERAL_RE = re.compile(r'"((?:[^"\\\n]|\\.)*)"')
FORMAT_RE = re.compile(r"%(?:%|[-+ #0]*(?:\d+|\*)?(?:\.(?:\d+|\*))?(?:hh|h|ll|l|z|j|t|L)?[diouxXfFeEgGcsp]|.?)")
# Characters a translation may only contain if the English text does
GUARDED = set('"\\<>&\x1f')


class I18nError(Exception):
    pass


# ---- C source ----

def strip_comments(text):
    """Blank out comments, keeping strings and line numbers."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def c_unescape(s):
    """Bytes of a C string literal body, decoded as UTF-8."""
    out, i = bytearray(), 0
    simple = {"n": 10, "t": 9, "r": 13, "\\": 92, '"': 34, "'": 39, "?": 63,
              "a": 7, "b": 8, "f": 12, "v": 11}
    while i < len(s):
        c = s[i]
        if c != "\\":
            out += c.encode()
            i += 1
            continue
        e = s[i + 1]
        if e in simple:
            out.append(simple[e])
            i += 2
        elif e == "x":
            m = re.match(r"[0-9A-Fa-f]+", s[i + 2:])
            out.append(int(m.group(), 16) & 0xFF)
            i += 2 + len(m.group())
        elif e in "01234567":
            m = re.match(r"[0-7]{1,3}", s[i + 1:])
            out.append(int(m.group(), 8) & 0xFF)
            i += 1 + len(m.group())
        elif e in "uU":
            size = 4 if e == "u" else 8
            out += chr(int(s[i + 2:i + 2 + size], 16)).encode()
            i += 2 + size
        else:
            raise I18nError(f"unknown escape \\{e}")
    return out.decode("utf-8")


def c_literal(text):
    """A C string literal for text.  Octal escapes take at most three digits, so
    they never swallow the character after them; UTF-8 stays as is."""
    parts = []
    for ch in text:
        if ch in '"\\':
            parts.append("\\" + ch)
        elif ord(ch) < 0x20 or ord(ch) == 0x7F:
            parts.append(f"\\{ord(ch):03o}")
        else:
            parts.append(ch)
    return '"' + "".join(parts) + '"'


def extract():
    """Return {msgid: [locations]} in order of first appearance."""
    found = {}
    files = sorted(list(SRC_DIR.glob("*.c")) + list(SRC_DIR.glob("*.h")))
    for path in files:
        text = strip_comments(path.read_text(encoding="utf-8"))
        for m in MACRO_RE.finditer(text):
            body = "".join(LITERAL_RE.findall(m.group(1)))
            try:
                msgid = c_unescape(body)
            except (I18nError, ValueError) as err:
                raise I18nError(f"{path.name}: {err}") from None
            if not msgid:
                continue
            line = text.count("\n", 0, m.start()) + 1
            found.setdefault(msgid, []).append(f"main/{path.name}:{line}")
    return found


# ---- PO files ----

def po_escape(s):
    return (s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
             .replace("\t", "\\t").replace("\x1f", "\\037"))


def po_unescape(s):
    return c_unescape(s)


def read_po(path):
    """Return (header dict, [entry]) where entry = {msgid, msgstr, fuzzy, line}."""
    entries, header = [], {}
    cur, field, flags = None, None, set()

    def finish():
        nonlocal cur
        if cur is not None:
            cur["fuzzy"] = "fuzzy" in flags
            if cur["msgid"] == "":
                for line in cur["msgstr"].splitlines():
                    if ":" in line:
                        k, v = line.split(":", 1)
                        header[k.strip()] = v.strip()
            else:
                entries.append(cur)
        cur = None

    for no, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line:
            finish()
            flags = set()
            field = None
        elif line.startswith("#~"):
            continue
        elif line.startswith("#,"):
            flags |= {f.strip() for f in line[2:].split(",")}
        elif line.startswith("#"):
            continue
        elif line.startswith("msgid "):
            finish()
            cur = {"msgid": "", "msgstr": "", "line": no}
            field = "msgid"
            cur[field] = po_unescape(line[6:].strip()[1:-1])
        elif line.startswith("msgstr "):
            field = "msgstr"
            cur[field] = po_unescape(line[7:].strip()[1:-1])
        elif line.startswith('"') and cur is not None and field:
            cur[field] += po_unescape(line[1:-1])
        else:
            raise I18nError(f"{path}:{no}: cannot read: {raw}")
    finish()
    return header, entries


def write_po(path, header, messages, translations):
    lines = ['msgid ""', 'msgstr ""']
    for k, v in header.items():
        lines.append(f'"{po_escape(k)}: {po_escape(v)}\\n"')
    for msgid, locations in messages.items():
        lines.append("")
        lines.append("#: " + " ".join(locations[:4]))
        lines.append(f'msgid "{po_escape(msgid)}"')
        lines.append(f'msgstr "{po_escape(translations.get(msgid, ""))}"')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def po_files():
    return sorted(p for p in PO_DIR.glob("*.po"))


def default_header(lang="", name=""):
    h = {"Content-Type": "text/plain; charset=UTF-8"}
    if lang:
        h["Language"] = lang
        h["X-Native-Name"] = name or lang
    return h


def cmd_update(_args):
    messages = extract()
    PO_DIR.mkdir(exist_ok=True)
    write_po(POT, default_header(), messages, {})
    for path in po_files():
        header, entries = read_po(path)
        old = {e["msgid"]: e["msgstr"] for e in entries if e["msgstr"]}
        gone = [m for m in old if m not in messages]
        write_po(path, header, messages, old)
        missing = sum(1 for m in messages if m not in old)
        print(f"{path.name}: {len(messages) - missing}/{len(messages)} translated"
              + (f", dropped {len(gone)} no longer used" if gone else ""))
        for m in gone:
            print(f"  dropped: {m!r}")
    print(f"{len(messages)} messages in {POT.relative_to(ROOT)}")


def cmd_init(args):
    path = PO_DIR / f"{args.lang}.po"
    if path.exists():
        raise I18nError(f"{path} already exists")
    PO_DIR.mkdir(exist_ok=True)
    write_po(path, default_header(args.lang, args.name), extract(), {})
    print(f"Created {path.relative_to(ROOT)}")


def cmd_stats(_args):
    messages = extract()
    for path in po_files():
        _h, entries = read_po(path)
        done = {e["msgid"] for e in entries if e["msgstr"] and not e["fuzzy"]}
        print(f"{path.stem}: {len(done & set(messages))}/{len(messages)}")


# ---- Tables ----

def fnv1a(text):
    h = 2166136261
    for b in text.encode("utf-8"):
        h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h


def formats(s):
    return [m.group() for m in FORMAT_RE.finditer(s)]


def check(msgid, msgstr, where):
    """Return msgstr made safe for the page, or raise I18nError."""
    # A plain ' could close the JavaScript string the text sits in.  If the
    # English text has one itself, it is markup (href='...') and stays.
    if "'" not in msgid:
        msgstr = msgstr.replace("'", "\u2019")
    if formats(msgid) != formats(msgstr):
        raise I18nError(f"{where}: % formats differ: {formats(msgid)} vs {formats(msgstr)}")
    bad = {c for c in msgstr if c in GUARDED and c not in msgid}
    if bad:
        raise I18nError(f"{where}: {''.join(sorted(bad))!r} not allowed in this translation")
    return msgstr


def cmd_gen(args):
    out = [
        "/* Generated by tools/i18n.py from i18n/<lang>.po.  Do not edit. */",
        '#include "i18n.h"',
        "",
    ]
    tables, errors = [], []
    for path in po_files():
        header, entries = read_po(path)
        lang = header.get("Language") or path.stem
        name = header.get("X-Native-Name") or lang
        rows = []
        for e in entries:
            if not e["msgstr"] or e["fuzzy"]:
                continue
            try:
                rows.append((fnv1a(e["msgid"]), e["msgid"],
                             check(e["msgid"], e["msgstr"], f"{path.name}:{e['line']}")))
            except I18nError as err:
                errors.append(str(err))
        rows.sort(key=lambda r: (r[0], r[1]))
        ident = re.sub(r"\W", "_", lang)
        out.append(f"static const i18n_entry_t s_{ident}[] = {{")
        for h, msgid, msgstr in rows:
            out.append(f"    {{ 0x{h:08x}u, {c_literal(msgid)},")
            out.append(f"      {c_literal(msgstr)} }},")
        if not rows:
            out.append("    { 0, \"\", \"\" },")
        out.append("};")
        out.append("")
        tables.append((lang, name, ident, len(rows)))

    if errors:
        for e in errors:
            print(f"i18n: {e}", file=sys.stderr)
        sys.exit(1)

    out.append("const i18n_table_t i18n_tables[] = {")
    for lang, name, ident, count in tables:
        out.append(f"    {{ {c_literal(lang)}, {c_literal(name)}, s_{ident}, {count} }},")
    if not tables:
        out.append('    { "", "", 0, 0 },')
    out.append("};")
    out.append(f"const int i18n_table_count = {len(tables)};")

    text = "\n".join(out) + "\n"
    target = Path(args.out)
    if not target.exists() or target.read_text(encoding="utf-8") != text:
        target.write_text(text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("update").set_defaults(fn=cmd_update)
    p = sub.add_parser("init")
    p.add_argument("lang")
    p.add_argument("name")
    p.set_defaults(fn=cmd_init)
    sub.add_parser("stats").set_defaults(fn=cmd_stats)
    p = sub.add_parser("gen")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_gen)
    args = parser.parse_args()
    try:
        args.fn(args)
    except I18nError as err:
        print(f"i18n: {err}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
