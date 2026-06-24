# C++ Call-Graph Tracing — Usage

Implements `implement-Doxyfile.md`. Fully offline: Doxygen + Python (stdlib only) + Graphviz.

## Logic
Each line in `command.txt` names a function the **test tool** (`testtool/main.cpp`)
defines. The tracer finds that test-tool function (the entry), follows the first
hop into DBM, then descends recursively through every DBM method called, applying
the break rules. Output = the DBM call subtree exercised by that command.

## Files
- `Doxyfile` — indexes BOTH `testtool/` (entry points) and `com/project/DBM`
  (call targets); XML-only output tuned for call-graph extraction (§5.2).
- `tools/callgraph_trace.py` — parses Doxygen XML, matches each command to its
  test-tool entry function (`TESTTOOL_KEYWORD`), walks the call tree applying the
  four break rules (§6), emits one `.dot` per command.
- `run_callgraph.sh` / `run_callgraph.bat` — chains Doxygen → trace → Graphviz.
- `command.txt` — one command (entry-function name/substring) per line.

## Prerequisites (not currently installed on this machine)
- `doxygen`  — e.g. `choco install doxygen.install`
- `graphviz` — e.g. `choco install graphviz` (provides `dot`)
- Python 3.8+ on PATH (the trace script uses only the standard library).

## Run
```sh
# from outscope/
./run_callgraph.sh svg        # or: run_callgraph.bat svg   (png / pdf also valid)
```
Outputs land in `callgraph_output/<command>.dot` and `.<format>`.

## Tuning (constants at top of tools/callgraph_trace.py)
- `DBM_KEYWORD` — break rule 2 in-project test (default `"DBM"`).
- `STDLIB_PREFIXES` / `STDLIB_NAMES` / `STDLIB_REGEXES` — break rule 3 (stdlib) skip list.
- `defaultmethod.txt` — break rule 3 (default methods): one name per line treated
  as a leaf. C++ special members (ctor `Class::Class`, dtor `~Class`) are detected
  automatically; add your own functions here. Override path with `--defaults`.
- `MAX_NODES` — break rule 4: stop once the traced tree reaches this many nodes
  (default `8`; entry counts as node 1). `MAX_DEPTH` is a secondary recursion guard.
- `MATCH_MODE` — `exact` | `substring` | `regex` command→function matching.

## Note
`command.txt` is currently empty — populate it with the entry-point function names
(or substrings) you want traced before running.
