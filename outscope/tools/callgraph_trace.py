#!/usr/bin/env python3
"""C++ call-graph tracer.
Reads Doxygen XML (produced from the project Doxyfile) plus a command list,
walks the call tree per command applying the break rules, and emits one
Graphviz DOT file per command. See implement-Doxyfile.md sections 4 and 6.
Usage:
    python callgraph_trace.py \
        --xml doxygen_out/xml \
        --commands command.txt \
        --out callgraph_output
Rendering to SVG/PNG is done separately by Graphviz `dot` (see run_callgraph.sh).
"""
from __future__ import annotations
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
# ---------------------------------------------------------------------------
# Configurable constants (tune here without touching logic) — section 6.
# ---------------------------------------------------------------------------
# Break rule 2: a callee is "inside the project" only if its source file path
# contains this keyword. The test tool's first hop into DBM is always allowed.
DBM_KEYWORD = "DBM"
# Entry points live in the test tool. A command line in command.txt matches a
# function *defined in a file whose path contains this keyword* (the test tool),
# never an identically-named method inside DBM. This is what anchors the walk at
# testtool/main.cpp rather than starting inside DBM.
TESTTOOL_KEYWORD = "testtool"
# Break rule 3: skip callees whose (qualified) name starts with any of these,
# or matches one of the exact stdlib names. Extend as needed for hand-rolled
# wrappers that look like user code.
STDLIB_PREFIXES = ("std::", "boost::", "cv::", "operator", "__")
STDLIB_NAMES = {
    "printf", "fprintf", "sprintf", "snprintf", "scanf",
    "malloc", "calloc", "realloc", "free",
    "memcpy", "memmove", "memset", "strcpy", "strncpy", "strlen", "strcmp",
    "pthread_create", "pthread_join", "pthread_mutex_lock", "pthread_mutex_unlock",
    "abs", "min", "max", "sqrt", "pow", "floor", "ceil", "round",
}
# pthread_* and similar families.
STDLIB_REGEXES = (re.compile(r"^pthread_\w+$"),)
# Break rule 4: stop once the traced tree reaches this many nodes (the entry
# function counts as node 1). The DBM pipeline (e.g. ADCensusStereo::Match) is
# far larger than the old test-tool-anchored trees, so the default is raised;
# override per run with --max-nodes.
MAX_NODES = 50
# Secondary safety net against pathological recursion depth.
MAX_DEPTH = 50
# Break rule 3 (default methods): names listed in this file are treated as
# leaves and not descended into. Resolved relative to the commands file / CWD.
DEFAULTMETHOD_FILE = "defaultmethod.txt"
# Command -> entry function matching mode: "exact" | "substring" | "regex".
# "exact" matches the function short name (or fully-qualified name) precisely, so a
# command like "Refine" does NOT also match "MultiStepRefiner::OutlierDetection"
# (whose class name merely *contains* "Refine"). Use "substring"/"regex" only when
# command.txt holds partial names.
MATCH_MODE = "exact"
# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------
@dataclass
class Func:
    refid: str
    name: str          # short name, e.g. "ComputeCost"
    qname: str         # qualified name, e.g. "ADCensusStereo::ComputeCost"
    file: str          # declaration file (location/@file)
    bodyfile: str = "" # definition file (location/@bodyfile) — where the body lives
    calls: list[str] = field(default_factory=list)  # refids it references/calls
# Populated at startup from DEFAULTMETHOD_FILE. Short names / qualified names
# that should be treated as leaves (break rule 3).
DEFAULT_METHODS: set[str] = set()
def load_default_methods(path: str) -> set[str]:
    """Read the user-supplied default-method break list."""
    names: set[str] = set()
    if not path or not os.path.isfile(path):
        return names
    with open(path, encoding="utf-8") as fh:
        for ln in fh:
            ln = ln.strip()
            if ln and not ln.startswith("#"):
                names.add(ln)
    return names
def is_cpp_special_member(fn: "Func") -> bool:
    """C++ compiler-default methods: ctor (name == class), dtor (~Class)."""
    if fn.name.startswith("~"):
        return True
    # Qualified name like "Ns::Class::Class" -> ctor.
    parts = fn.qname.split("::")
    return len(parts) >= 2 and parts[-1] == parts[-2]
def is_default_method(fn: "Func") -> bool:
    """Break rule 3 (default methods): special members or listed names."""
    if is_cpp_special_member(fn):
        return True
    return fn.name in DEFAULT_METHODS or fn.qname in DEFAULT_METHODS
def is_stdlib(name: str) -> bool:
    """Break rule 3 (stdlib / language helpers)."""
    if name.startswith(STDLIB_PREFIXES):
        return True
    if name in STDLIB_NAMES:
        return True
    return any(rx.match(name) for rx in STDLIB_REGEXES)
def in_project(file_path: str) -> bool:
    """Break rule 2 — file path lives under the DBM tree (not the test tool)."""
    fp = (file_path or "").lower()
    return DBM_KEYWORD.lower() in fp and TESTTOOL_KEYWORD.lower() not in fp
# ---------------------------------------------------------------------------
# 4.1 Load Doxygen XML
# ---------------------------------------------------------------------------
def load_xml(xml_dir: str) -> dict[str, Func]:
    """Parse every compound XML file and build refid -> Func."""
    index_path = os.path.join(xml_dir, "index.xml")
    if not os.path.isfile(index_path):
        sys.exit(f"[fatal] {index_path} not found — run Doxygen first.")
    funcs: dict[str, Func] = {}
    for fname in os.listdir(xml_dir):
        if not fname.endswith(".xml") or fname in ("index.xml", "Doxyfile.xml"):
            continue
        path = os.path.join(xml_dir, fname)
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            print(f"[warn] skip unparseable {fname}: {e}", file=sys.stderr)
            continue
        for compound in root.findall("compounddef"):
            comp_name = compound.findtext("compoundname") or ""
            for member in compound.iter("memberdef"):
                if member.get("kind") != "function":
                    continue
                refid = member.get("id")
                if not refid:
                    continue
                short = member.findtext("name") or ""
                loc = member.find("location")
                src = loc.get("file") if loc is not None else ""
                bodysrc = loc.get("bodyfile") if loc is not None else ""
                qname = f"{comp_name}::{short}" if comp_name and "::" not in short else short
                # <references> elements carry the callee refids.
                calls = [r.get("refid") for r in member.findall("references")
                         if r.get("refid")]
                funcs[refid] = Func(refid=refid, name=short, qname=qname,
                                    file=src or "", bodyfile=bodysrc or "",
                                    calls=calls)
    return funcs
# ---------------------------------------------------------------------------
# 4.2 Match each command to entry function(s)
# ---------------------------------------------------------------------------
def in_testtool(fn: "Func") -> bool:
    """A command names a function *in the test tool* (testtool/DBMDownLoadTest.cpp).
    The walk enters that exact function, then descends into whichever callees
    live in DBM (break rule 2). Doxygen records the declaration in location/@file
    and the definition in location/@bodyfile, so we accept either."""
    kw = TESTTOOL_KEYWORD.lower()
    return kw in (fn.bodyfile or "").lower() or kw in (fn.file or "").lower()
def match_entries(command: str, funcs: dict[str, Func]) -> list[Func]:
    if MATCH_MODE == "exact":
        pred = lambda f: f.name == command or f.qname == command
    elif MATCH_MODE == "regex":
        rx = re.compile(command)
        pred = lambda f: bool(rx.search(f.qname))
    else:  # substring
        c = command.lower()
        pred = lambda f: c in f.name.lower() or c in f.qname.lower()
    matches = [f for f in funcs.values() if pred(f)]
    # The command is a test-tool function name. Prefer the test-tool definition
    # over an identically-named DBM method, so the walk starts at the harness
    # function and the first hop reveals the DBM call it exercises.
    tt = [f for f in matches if in_testtool(f)]
    return tt or matches
# ---------------------------------------------------------------------------
# 4.3 Walk the call graph with break rules + 4.4 collect nodes/edges
# ---------------------------------------------------------------------------
class Walk:
    def __init__(self, funcs: dict[str, Func]):
        self.funcs = funcs
        self.nodes: dict[str, Func] = {}       # refid -> Func (visited)
        self.edges: set[tuple[str, str]] = set()
        self.recursion_edges: set[tuple[str, str]] = set()
    def run(self, entry: Func):
        self.nodes[entry.refid] = entry
        self._descend(entry, path=[entry.refid], depth=0, is_entry_level=True)
    def _descend(self, fn: Func, path: list[str], depth: int, is_entry_level: bool):
        # Secondary safety net — pathological depth.
        if depth >= MAX_DEPTH:
            return
        for callee_id in fn.calls:
            # Break rule 4 — node cap. Stop once the tree has MAX_NODES nodes.
            if len(self.nodes) >= MAX_NODES:
                return
            callee = self.funcs.get(callee_id)
            if callee is None:
                continue  # external / unresolved reference
            # Break rule 3 — stdlib / language helpers.
            if is_stdlib(callee.qname) or is_stdlib(callee.name):
                continue
            # Break rule 3 — default methods (C++ special members + listed names).
            if is_default_method(callee):
                continue
            # Break rule 2 — outside DBM. The entry level's first hop is allowed.
            if not is_entry_level and not in_project(callee.file):
                continue
            # Break rule 1 — recursion (callee already on current path).
            if callee_id in path:
                self.nodes.setdefault(callee_id, callee)
                self.recursion_edges.add((fn.refid, callee_id))
                continue
            self.nodes.setdefault(callee_id, callee)
            self.edges.add((fn.refid, callee_id))
            self._descend(callee, path + [callee_id], depth + 1, is_entry_level=False)
# ---------------------------------------------------------------------------
# 4.4 Emit DOT
# ---------------------------------------------------------------------------
def sanitize(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_") or "cmd"
def emit_dot(command: str, entries: list[Func], walk: Walk) -> str:
    entry_ids = {e.refid for e in entries}
    lines = [f'digraph "{command}" {{',
             '  rankdir=LR;',
             '  node [shape=box, fontname="Helvetica", fontsize=10];',
             '  edge [fontname="Helvetica", fontsize=8];']
    for refid, fn in sorted(walk.nodes.items(), key=lambda kv: kv[1].qname):
        label = fn.qname.replace('"', r'\"')
        if refid in entry_ids:
            lines.append(f'  "{refid}" [label="{label}", '
                         'style="filled,bold", fillcolor="#ffe08a"];')
        else:
            lines.append(f'  "{refid}" [label="{label}"];')
    for a, b in sorted(walk.edges):
        lines.append(f'  "{a}" -> "{b}";')
    for a, b in sorted(walk.recursion_edges):
        lines.append(f'  "{a}" -> "{b}" '
                     '[color="red", style="dashed", label="recursion"];')
    lines.append("}")
    return "\n".join(lines) + "\n"
# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def read_commands(path: str) -> list[str]:
    with open(path, encoding="utf-8") as fh:
        return [ln.strip() for ln in fh if ln.strip()]
def main(argv=None):
    global MAX_NODES
    ap = argparse.ArgumentParser(description="C++ call-graph tracer (Doxygen XML -> DOT).")
    ap.add_argument("--xml", default="doxygen_out/xml", help="Doxygen XML directory.")
    ap.add_argument("--commands", default="command.txt", help="Command list file.")
    ap.add_argument("--out", default="callgraph_output", help="Output folder for .dot files.")
    ap.add_argument("--defaults", default=DEFAULTMETHOD_FILE,
                    help="File listing default methods to treat as leaves.")
    ap.add_argument("--max-nodes", type=int, default=MAX_NODES,
                    help="Break rule 4: stop a tree once it reaches this many "
                         f"nodes (default {MAX_NODES}).")
    args = ap.parse_args(argv)
    MAX_NODES = args.max_nodes
    global DEFAULT_METHODS
    DEFAULT_METHODS = load_default_methods(args.defaults)
    print(f"[info] loaded {len(DEFAULT_METHODS)} default-method names "
          f"from {args.defaults}")
    funcs = load_xml(args.xml)
    print(f"[info] loaded {len(funcs)} functions from {args.xml}")
    commands = read_commands(args.commands)
    if not commands:
        print(f"[warn] {args.commands} is empty — nothing to trace.")
        return 0
    os.makedirs(args.out, exist_ok=True)
    for command in commands:
        entries = match_entries(command, funcs)
        walk = Walk(funcs)
        for e in entries:
            walk.run(e)
        dot = emit_dot(command, entries, walk)
        out_path = os.path.join(args.out, f"{sanitize(command)}.dot")
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(dot)
        # Run log — section 8.
        print(f"[cmd] {command!r}: entries={len(entries)} "
              f"nodes={len(walk.nodes)} edges={len(walk.edges)} "
              f"recursion={len(walk.recursion_edges)} -> {out_path}")
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
