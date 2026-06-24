# C++ Call Graph Tracing — Implementation Plan

## 1. Objective

Build a tool that produces a **visual call graph** for each command in `command.txt`.
For every command, the graph must show the chain of function calls starting from the test tool's entry point, descending into the project source under `/com/project/DBM`, and stopping cleanly at well-defined boundaries.

The end result: one diagram per command, saved as SVG/PNG, so engineers can audit which DBM functions are exercised by each test command.

---

## 2. Scope and Constraints

- **Private project** — every tool used must be fully offline and open source.
- **Input** — `command.txt`, one command name per line.
- **Tracing root** — only function calls landing in `/com/project/DBM` are followed.
- **Output** — one graph file per command, in a dedicated output folder.
- **No AI services, no cloud upload, no telemetry.**

---

## 3. Tool Stack

| Stage | Tool | Purpose |
|---|---|---|
| Parse source | **Doxygen** | Static analysis → produces XML index of all functions and their references |
| Build graph data | **Custom Python script** | Parse Doxygen XML, walk the call tree per command, apply break rules |
| Render graph | **Graphviz (`dot`)** | Convert the generated DOT files to SVG/PNG/PDF |

All three tools are local, open source, and run with no network access.

---

## 4. Pipeline Overview

```
command.txt
     │
     ▼
[ Doxygen ]  ── parses C++ source ──► doxygen_out/xml/*.xml
     │
     ▼
[ Trace script ] ── reads XML + command list ──► one .dot file per command
     │
     ▼
[ Graphviz dot ] ── renders ──► one .svg/.png per command
```

---

## 5. Implementation Steps

### Step 1 — Prepare the environment
- Install Doxygen, Graphviz on the build machine.
- Confirm the project source tree is accessible and contains the `/com/project/DBM` folder.
- Place `command.txt` in the working directory D:\dev\Autonomous_AI_robot\outscope\command.txt. 

### Step 2 — Configure Doxygen for XML output
Create a `Doxyfile` tuned for call graph extraction. The important configuration:
- `INPUT` points at the project source root.
- `RECURSIVE = YES` so subfolders (including DBM) are indexed.
- `GENERATE_XML = YES` — XML is the input format for the trace script.
- `EXTRACT_ALL = YES`, `EXTRACT_PRIVATE = YES`, `EXTRACT_STATIC = YES` — make sure no functions are skipped.
- `SOURCE_BROWSER = YES`, `REFERENCES_RELATION = YES`, `REFERENCED_BY_RELATION = YES` — these flags are what make Doxygen emit the cross-reference information (caller → callee) that the trace script needs.
- Exclude `build/`, `third_party/`, and other irrelevant folders.

### Step 3 — Run Doxygen
Run Doxygen against the project. The output directory will contain an `xml/` subfolder with one XML file per compound (class/file/namespace) plus an `index.xml` master index.

### Step 4 — Build the trace script
The script handles four responsibilities:

**4.1 Load Doxygen XML**
Walk the `xml/` directory and parse each compound file. For every function, capture:
- its qualified name,
- the source file it lives in,
- the list of functions it calls (from the `<references>` tags in Doxygen XML).

Store this in an in-memory map: `function_name → { file, calls[] }`.

**4.2 Match each command to an entry function**
For each line in `command.txt`, search the function map for matching entry-point functions in the test tool. Matching can be by exact name or substring depending on naming convention.

**4.3 Walk the call graph with break rules**
Starting from the entry function, recursively descend through `calls[]`. At each step, apply the four break rules from section 6 before recording the edge and continuing.

**4.4 Emit a DOT file per command**
Produce a Graphviz DOT file containing:
- one node per function visited,
- one directed edge per call,
- highlight styling for entry-point nodes,
- distinct styling (red, dashed, label "recursion") for any recursion edges.

### Step 5 — Render with Graphviz
Run `dot -Tsvg` (or `-Tpng`) on each DOT file. Save outputs in a dedicated folder, named after the command.

### Step 6 — Wrap as a pipeline script
A single shell script chains steps 3, 4, and 5 so the engineer runs one command and gets all graphs.

---

## 6. Break Rules (Termination Conditions)

The recursive walk must stop on **any** of these conditions. This is the heart of the design and must be enforced rigorously.

| # | Rule | Action when triggered |
|---|---|---|
| 1 | **Recursion detected** — the function is already on the current walk path | Add a red dashed edge labeled "recursion", do not descend further |
| 2 | **Callee is outside `/com/project/DBM`** — its source file path does not contain the DBM keyword (applied only after the entry level, so the test tool's first hop into DBM is allowed) | Skip the callee entirely, do not draw an edge |
| 3 | **Callee is a language default / standard library function** — name starts with `std::`, `boost::`, `operator`, `__`, or matches the configured stdlib prefix list (`printf`, `malloc`, `memcpy`, `pthread_*`, etc.) | Skip the callee entirely, do not draw an edge |
| 4 | **Max depth exceeded** — safety net to prevent runaway graphs | Stop the branch silently |

The configurable items (DBM keyword, stdlib prefix list, max depth) live as constants at the top of the trace script so they can be tuned without touching logic.

---

## 7. Inputs

- **`command.txt`** — one command per line, blank lines and whitespace ignored.
- **`Doxyfile`** — Doxygen configuration tuned per section 5.2.
- **Project source tree** — must contain `/com/project/DBM` and the test tool source.

---

## 8. Outputs

For each command, the pipeline produces:
- A `.dot` file (the graph in Graphviz source form).
- A rendered graph file (`.svg` by default; `.png` or `.pdf` selectable).

All outputs land in a single folder (e.g. `callgraph_output/`) named after the command, with characters unsafe for filenames sanitized.

A run log on stdout records, for each command:
- how many entry functions matched,
- how many call edges were recorded,
- whether rendering succeeded.

---

## 9. Verification Plan

After the first end-to-end run:
1. Open one or two SVGs and confirm the entry node corresponds to the expected test-tool function.
2. Spot-check that no nodes appear from outside `/com/project/DBM` (except the entry).
3. Spot-check that no `std::` or `printf`-class nodes appear.
4. Pick a function known to be recursive and confirm the red "recursion" edge is rendered.
5. Pick a command whose underlying function calls nothing in DBM — confirm a graph with only the entry node is produced (or a clear "no edges" log line).

---

## 10. Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Doxygen misses some calls (function pointers, virtual dispatch, templates) | Acknowledge this as a known limitation of static analysis; combine with runtime tracing (e.g. Callgrind) if exhaustive coverage is required |
| Command names in `command.txt` don't map cleanly to function names | Make the matching configurable (exact / substring / regex) |
| Project has hand-rolled wrappers around stdlib that look like user code | Extend the stdlib prefix list in the trace script |
| Graphs become unreadable for large commands | Add a depth cap and/or per-command node limit; offer PDF output for zooming |
| Doxygen run is slow on a large codebase | Doxygen is run only when source changes; cache the XML output |

---

## 11. Future Extensions

- **Per-file scope filter** — allow narrowing to a specific subfolder of DBM, not just DBM as a whole.
- **Multiple output formats in one run** — emit SVG and PNG simultaneously for documentation vs. quick preview.
- **HTML index page** — generate a single landing page listing every command with thumbnails and links to its graph.
- **Diff mode** — compare two runs (e.g. before/after refactor) and highlight added or removed edges.
- **Runtime cross-check** — pair with Callgrind output to flag functions Doxygen indexed but the test tool never actually executes (dead-call detection).

---

## 12. Deliverables Checklist

- [ ] `Doxyfile` configured per section 5.2
- [ ] Trace script implementing sections 4.1–4.4 and the break rules from section 6
- [ ] Shell pipeline script chaining Doxygen → trace → Graphviz
- [ ] `command.txt` populated with the target commands
- [ ] First successful run producing one graph per command in the output folder
- [ ] Verification checklist from section 9 signed off