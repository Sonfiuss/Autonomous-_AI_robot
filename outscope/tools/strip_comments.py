#!/usr/bin/env python3
"""Doxygen INPUT_FILTER: emit a comment-stripped, UTF-8 view of a C/C++ source.

Doxygen invokes this as `python strip_comments.py <file>` and parses whatever we
write to stdout (the original file is never modified). We do two jobs in one pass
so the call graph is reliable on the real DBM tree:

  1. Encoding — the DBM sources mix UTF-8 (adcensus/) with Shift-JIS / CP932
     (Japanese comments, e.g. DBMuim/, DBMstd/). We detect per file and decode,
     then emit UTF-8.

  2. Comment removal — Japanese comments are exactly what breaks extraction:
       * a `//` comment ending in a char whose 2nd byte is 0x5C ('\\', e.g. 表 ソ
         十 ・) is read as a line-continuation and swallows the next line;
       * a `/* */` doc block whose terminator is mis-read runs on until a later
         `*/` (e.g. an inline default-arg comment), swallowing the function
         signature — so the function vanishes from the XML and the command can't
         find it (this is why DeleteNonSpatialRecord was missing).
     Stripping all `//` and `/* */` comments removes the failure mode entirely;
     identifiers / call references (which are ASCII) are untouched.

Newlines inside comments are preserved so Doxygen line numbers stay accurate.
String and character literals are honored so comment markers inside them are
left alone.
"""

from __future__ import annotations

import sys


def decode_bytes(data: bytes) -> str:
    """UTF-8 if valid, otherwise CP932 (Windows Shift-JIS superset)."""
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        # 'replace' guarantees we never crash on a stray byte; identifiers are
        # ASCII so the call graph is unaffected by an imperfect comment glyph.
        return data.decode("cp932", errors="replace")


def strip_comments(src: str) -> str:
    out: list[str] = []
    i, n = 0, len(src)
    CODE, LINE, BLOCK, STR, CHAR = range(5)
    state = CODE
    while i < n:
        c = src[i]
        nxt = src[i + 1] if i + 1 < n else ""
        if state == CODE:
            if c == "/" and nxt == "/":
                state = LINE
                i += 2
            elif c == "/" and nxt == "*":
                state = BLOCK
                i += 2
            elif c == '"':
                out.append(c)
                state = STR
                i += 1
            elif c == "'":
                out.append(c)
                state = CHAR
                i += 1
            else:
                out.append(c)
                i += 1
        elif state == LINE:
            if c == "\\" and nxt == "\n":      # line-continuation: stay in comment
                out.append("\n")
                i += 2
            elif c == "\\" and nxt == "\r":
                out.append(src[i + 1:i + 3])   # preserve \r\n
                i += 3 if src[i + 2:i + 3] == "\n" else 2
            elif c == "\n":
                out.append("\n")
                state = CODE
                i += 1
            else:
                i += 1                         # drop comment char
        elif state == BLOCK:
            if c == "*" and nxt == "/":
                state = CODE
                i += 2
            elif c == "\n":
                out.append("\n")               # keep line count
                i += 1
            else:
                i += 1                         # drop comment char
        elif state == STR:
            out.append(c)
            if c == "\\":
                if nxt:
                    out.append(nxt)
                i += 2
            else:
                if c == '"':
                    state = CODE
                i += 1
        elif state == CHAR:
            out.append(c)
            if c == "\\":
                if nxt:
                    out.append(nxt)
                i += 2
            else:
                if c == "'":
                    state = CODE
                i += 1
    return "".join(out)


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        sys.stderr.write("usage: strip_comments.py <file>\n")
        return 2
    try:
        with open(argv[1], "rb") as fh:
            data = fh.read()
    except OSError as e:
        sys.stderr.write(f"strip_comments: {e}\n")
        return 1
    text = strip_comments(decode_bytes(data))
    sys.stdout.buffer.write(text.encode("utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
