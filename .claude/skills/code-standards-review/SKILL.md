---
name: code-standards-review
description: Review just-written or just-edited code against this project's style/structure standards (naming, comments, const usage, OOP structure, error handling, debug logging), then auto-fix and re-review until it passes. Invoke manually after finishing a coding task, before marking a task "done" in the task lifecycle protocol. Covers C++ modules (motivation/, stereo-camera/) and Python modules (simulation/, depth-anything/).
---

# Code Standards Review

Run this after a coding task is functionally complete, before handing it back to the user. Goal: catch style/structure issues the way a strict human reviewer would, fix them, and confirm the result — not just report problems and stop.

**Where this fits in the flow:** run `/code-standards-review` FIRST (it may move/rename code), then `/code-logic-review` second. Running logic review first risks a later style refactor reintroducing a logic concern. Both are the gate before a task moves to `done` in the task lifecycle protocol.

## Scope

Only review files touched in the current task (check `git diff` / recently edited files), not the whole repo, unless the user explicitly asks for a full-repo pass.

Detect language per file:
- `.cpp` / `.hpp` / `.h` / `.cc` → C++ rules
- `.py` → Python rules

## Checklist

### 1. Naming consistency
- Function/variable names follow the convention already dominant in that file (don't introduce a new casing style mid-file).
- No ambiguous abbreviations (`tmp2`, `val_x`) unless already the file's convention.
- C++: types/classes in PascalCase, functions in camelCase or snake_case matching surrounding code, member variables with a consistent prefix/suffix (e.g. `m_` or trailing `_`) if the file already uses one.
- Python: `snake_case` for functions/variables, `PascalCase` for classes, `UPPER_CASE` for module-level constants (PEP 8).

### 2. Comments
- Every public/exported function or class has one short comment above it stating *what it does* (not a multi-line docstring unless the file already uses that style) — only if the name doesn't already make it obvious.
- No comments restating the obvious line below them.
- No leftover commented-out dead code.

### 3. Constants, not magic values
- No raw magic numbers/strings inline in logic (thresholds, buffer sizes, topic names, ports, timeouts).
- C++: declared as `constexpr`/`const`/`#define` near the top of the file or in a dedicated header, referenced by name at call sites — never inlined as a literal in the middle of a function.
- Python: declared as `UPPER_CASE` module-level constants, referenced by name.

### 4. Structure: classes/structs over loose functions
- If a task involves 3+ functions that share state (config, buffers, handles), that state should be grouped into a `struct`/`class`/`dataclass`, not passed around as separate loose function args or globals.
- C++: use `struct` for plain data bags, `class` when there's behavior + invariants to protect.
- Python: use `@dataclass` or a class for grouped attributes instead of a bag of module-level variables or a tuple/dict being passed everywhere.

### 5. OOP practices (apply where the design actually calls for it — don't force inheritance/overload where a plain function is clearer)
- **Encapsulation**: internal state not exposed raw when it has invariants to protect (C++: private + accessors; Python: `_name` convention + property if validation is needed).
- **Inheritance**: used only for genuine is-a relationships with shared behavior, not to avoid duplicating two lines.
- **Overloading/overriding**: C++ overloads are unambiguous (no surprising implicit conversions); overrides use `override` keyword. Python: no misleading same-name methods with incompatible signatures.
- Don't flag missing OOP where the existing codebase style is intentionally procedural (e.g. simple ROS/ZMQ callback scripts) — match the file's existing paradigm first.

### 6. Error handling
- Return values from I/O, UART, serial, socket, file, and subprocess calls are checked, not ignored.
- No empty `catch`/`except` blocks swallowing errors silently.
- Null/None pointers or optional values checked before dereference/use.
- Resource cleanup (files, sockets, camera handles) happens even on the error path.

### 7. Debug-mode logging (compiled/stripped in release)
- C++: debug logs wrapped in a macro gate, e.g.:
  ```cpp
  #ifdef DEBUG_MODE
  #define DLOG(...) fprintf(stderr, __VA_ARGS__)
  #else
  #define DLOG(...)
  #endif
  ```
  Used at function entry for non-trivial functions and around key calculations — never raw `printf`/`std::cout` debug spam left in release-path code.
- Python: use the standard `logging` module at `DEBUG` level (`logger.debug(...)`), not `print()`, so it's silenced by the app's log-level config in production instead of hardcoded on/off. This is the Python equivalent of the C++ macro gate — flag raw `print()` used for debug tracing as a violation.

## Process

1. Identify the files changed in this task (`git diff --name-only` or the files you just edited).
2. Read each file fully — don't review from memory of the edit.
3. For each file, check every item in the Checklist above. Note concrete violations with file:line.
4. If violations found: **fix them directly** (Edit tool), keeping fixes minimal and scoped to what's flagged — do not refactor unrelated code.
5. Re-run the checklist against the fixed file.
6. Repeat steps 3–5 until a pass finds zero violations, or up to 3 fix iterations. If still failing after 3 iterations, stop and report the remaining issues to the user instead of continuing to loop.
7. Report a short summary: files reviewed, violations found, violations fixed, and any that couldn't be auto-fixed (e.g. require a design decision from the user, like whether to add a new class).
8. Append one line to the active task file's Execution log recording the outcome, e.g. `[HH:MM] standards-review: N violations fixed, M deferred | info: <key finding>` — this ties the review into the task lifecycle before the task moves to `done`.

Do not ask for confirmation before applying fixes in this loop — the fix-then-reverify loop is the point of this skill. Do ask the user if a fix would require a structural decision beyond straightforward style correction (e.g. "should this become its own class, or is it fine as a free function?").
