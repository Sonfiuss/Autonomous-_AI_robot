# coding: utf-8
"""
run_command.py

Workflow:
  1. Read command names from command.txt (one per line)
  2. For each command, grep g_funcs in DBMDownLoadTest.cpp to resolve the C++ function name
  3. Extract the function body from the test file
  4. List all method calls inside the body as:  ClassName-MethodName(args)

command.txt format (one command per line, blank lines / # comments ignored):
    CreateDataset
    DeleteRecord

Output format:
    [CreateDataset -> CreateDataSet]
      DBMstdDatabase-Connect(host, port)
      DBMstdConditionData-SetConditionBaseList(1)

Usage:
    python tools/run_command.py --cmd  testtool/command.txt \
                                --src  testtool/DBMDownLoadTest.cpp
"""

import os
import re
import argparse
import sys
sys.path.insert(0, os.path.dirname(__file__))
from analyze_function import _build_member_typemap


# ---------------------------------------------------------------------------
# Patterns
# ---------------------------------------------------------------------------

# g_funcs[_T("CommandName")] = FuncName;
RE_GFUNC = re.compile(
    r'g_funcs\s*\[\s*_T\s*\(\s*["\'](\w+)["\']\s*\)\s*\]\s*=\s*(\w+)\s*;'
)

# Function name on a line — handles plain FuncName( and ClassName::FuncName(
RE_FUNC_NAME = re.compile(r'^\s*(?:[\w*&:<>]+\s+)+(?:(\w+)::)?(\w+)\s*\(')

# Method call:  obj.method(args)  obj->method(args)  obj[i]->method(args)
RE_OBJ_CALL = re.compile(
    r'(\w+)'
    r'(?:\[[^\]]*\])*'          # optional subscript(s)
    r'\s*(?:->|\.)\s*'
    r'(\w+)'
    r'\s*\(([^)]*)\)'
)

IGNORE_FUNCS = {'Format', '_T', 'LogError', 'MONITOR_ERROR0', 'AfxMessageBox',
                'TRACE', 'ASSERT', 'VERIFY', 'DEBUG_NEW'}

# Direct call — any identifier, no preceding .  ->  ::
RE_DIRECT_CALL = re.compile(
    r'(?<![.>:\w])([A-Za-z_]\w*)\s*\(([^)]*)\)'
)

# new ClassName(args)
RE_NEW_CALL = re.compile(r'\bnew\s+(\w+)\s*\(([^)]*)\)')

# Chained return-value call:  expr()->method(args)  or  expr().method(args)
RE_CHAIN_CALL = re.compile(r'\)\s*(?:->|\.)\s*(\w+)\s*\(([^)]*)\)')

# Template function call:  FuncName<Type>(args)
RE_TEMPLATE_CALL = re.compile(r'(?<![.>:\w])([A-Za-z_]\w*)\s*<[^>()]*>\s*\(([^)]*)\)')

# Variable declaration:  Type name;  Type* name;  Type *name;  Type *&name = ...
RE_VAR_DECL = re.compile(
    r'^\s*([\w:<>]+)'       # base type  (group 1)
    r'[\s*&]+'              # separator: spaces, *, & in any combo
    r'(\w+)'                # variable name  (group 2)
    r'\s*(?:;|=|\(|\{)'
)

SKIP_KEYWORDS = {
    'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'return',
    'delete', 'new', 'throw', 'catch', 'try', 'sizeof', 'nullptr',
    'true', 'false', 'NULL', 'void', 'int', 'bool', 'char', 'short',
    'long', 'float', 'double', 'unsigned', 'signed', 'auto',
    # common stdlib / stl names that appear as direct calls
    'static_cast', 'dynamic_cast', 'reinterpret_cast', 'const_cast',
    'std', 'size', 'empty', 'begin', 'end', 'data', 'get', 'make_shared',
    'make_unique', 'move', 'forward', 'decltype', 'sizeof', 'alignof',
}

def _is_macro(name: str) -> bool:
    """Return True if name looks like an ALL_CAPS macro (e.g. MONITOR_ERROR0)."""
    return bool(re.match(r'^[A-Z][A-Z0-9_]+$', name))


# ---------------------------------------------------------------------------
# Step 1 — read commands
# ---------------------------------------------------------------------------

def read_commands(path: str) -> list:
    commands = []
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith('#'):
                    commands.append(line)
    except FileNotFoundError:
        print(f'[run_command] command.txt not found: {path}')
    return commands


# ---------------------------------------------------------------------------
# Step 2 — build g_funcs map from test file
# ---------------------------------------------------------------------------

def build_gfuncs_map(src_path: str) -> dict:
    """Return {'CommandName': 'CppFuncName', ...}"""
    mapping = {}
    try:
        with open(src_path, encoding='utf-8', errors='replace') as fh:
            for line in fh:
                m = RE_GFUNC.search(line)
                if m:
                    cmd_name  = m.group(1)
                    func_name = m.group(2)
                    mapping[cmd_name] = func_name
    except FileNotFoundError:
        print(f'[run_command] source file not found: {src_path}')
    return mapping


# ---------------------------------------------------------------------------
# Step 3 — extract function body from test file
# ---------------------------------------------------------------------------

def extract_body(func_name: str, src_path: str):
    """Return (params_str, [body_lines]) or None.

    Handles params that span multiple lines or contain nested parens/templates.
    Strategy:
      1. Find a line where 'FuncName(' appears (not preceded by . -> ::)
      2. Collect all text until matching closing paren -> that is params_str
      3. Scan forward for the opening '{' of the body
      4. Collect body lines by brace depth
    """
    try:
        with open(src_path, encoding='utf-8', errors='replace') as fh:
            lines = fh.readlines()
    except FileNotFoundError:
        return None

    n = len(lines)
    i = 0
    while i < n:
        line = lines[i]
        m = RE_FUNC_NAME.match(line)
        if m and m.group(2) == func_name:
            # Collect full text from opening paren to matching closing paren
            full_text = ''.join(lines[i:min(i + 20, n)])
            open_idx = full_text.index('(')
            depth = 0
            close_idx = None
            for k, ch in enumerate(full_text[open_idx:], start=open_idx):
                if ch == '(':
                    depth += 1
                elif ch == ')':
                    depth -= 1
                    if depth == 0:
                        close_idx = k
                        break
            if close_idx is None:
                i += 1
                continue
            params_str = full_text[open_idx + 1:close_idx]

            # Find opening brace of body (after close_idx in full_text)
            after_sig = full_text[close_idx + 1:]
            brace_pos = after_sig.find('{')
            if brace_pos == -1:
                i += 1
                continue

            # Figure out which line the '{' is on
            chars_to_brace = close_idx + 1 + brace_pos
            consumed = 0
            brace_line_idx = i
            for li in range(i, min(i + 20, n)):
                consumed += len(lines[li])
                if consumed > chars_to_brace:
                    brace_line_idx = li
                    break

            # Now collect body starting after the '{'
            brace_depth = 1
            body = []
            # Rest of brace_line after the '{'
            bl = lines[brace_line_idx]
            brace_col = bl.index('{') if '{' in bl else 0
            rest = bl[brace_col + 1:]
            brace_depth += rest.count('{') - rest.count('}')
            body.append(rest)

            j = brace_line_idx + 1
            while j < n and brace_depth > 0:
                l = lines[j]
                brace_depth += l.count('{') - l.count('}')
                body.append(l)
                j += 1
            class_name = m.group(1) or ''
            return params_str, body, class_name
        i += 1
    return None


# ---------------------------------------------------------------------------
# Step 4 — extract calls inside the body
# ---------------------------------------------------------------------------

def _clean_type(raw: str) -> str:
    return raw.replace('const', '').replace('*', '').replace('&', '') \
               .lstrip(':').strip()


def _seed_type_map(params_str: str) -> dict:
    type_map = {}
    for param in params_str.split(','):
        param = param.split('=')[0].strip()
        tokens = param.split()
        if len(tokens) < 2:
            continue
        var_name = tokens[-1].lstrip('*&').rstrip('[]').strip()
        raw_type = _clean_type(' '.join(tokens[:-1]))
        if var_name and re.match(r'^\w+$', var_name):
            type_map[var_name] = raw_type
    return type_map


def extract_calls(params_str: str, body_lines: list, class_name: str = '',
                  src_root: str = '') -> list:
    type_map = _seed_type_map(params_str)
    # Seed member variable types from the class header so obj->method() on members resolves
    if class_name and src_root:
        type_map.update(_build_member_typemap(class_name, src_root))
    calls = []

    for line in body_lines:
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('*') \
                or stripped.startswith('/*'):
            continue

        # Track local variable declarations
        var_m = RE_VAR_DECL.match(line)
        if var_m:
            raw_type = _clean_type(var_m.group(1))
            var_name = var_m.group(2).strip()
            if re.match(r'^[A-Z]\w*$', raw_type) and re.match(r'^\w+$', var_name):
                type_map[var_name] = raw_type

        # new ClassName(args) — constructor call
        for nm in RE_NEW_CALL.finditer(line):
            cls  = nm.group(1).strip()
            args = nm.group(2).strip()
            if cls not in SKIP_KEYWORDS and not _is_macro(cls) and cls not in IGNORE_FUNCS:
                entry = f'{cls}-{cls}({args})'
                if entry not in calls:
                    calls.append(entry)

        # obj.method(args)  obj->method(args)  obj[i]->method(args)
        for cm in RE_OBJ_CALL.finditer(line):
            obj  = cm.group(1)
            meth = cm.group(2)
            args = cm.group(3).strip()
            if meth in SKIP_KEYWORDS or _is_macro(meth) or meth in IGNORE_FUNCS:
                continue
            resolved = type_map.get(obj)
            if resolved:
                entry = f'{resolved}-{meth}({args})'
                if entry not in calls:
                    calls.append(entry)

        # Chained return-value calls:  expr()->method(args)  expr().method(args)
        for cm in RE_CHAIN_CALL.finditer(line):
            meth = cm.group(1).strip()
            args = cm.group(2).strip()
            if meth in SKIP_KEYWORDS or _is_macro(meth) or meth in IGNORE_FUNCS:
                continue
            entry = f'-{meth}({args})'
            if entry not in calls:
                calls.append(entry)

        # Template function calls:  Func<Type>(args)
        for tm in RE_TEMPLATE_CALL.finditer(line):
            meth = tm.group(1).strip()
            args = tm.group(2).strip()
            if meth in SKIP_KEYWORDS or _is_macro(meth) or meth in IGNORE_FUNCS:
                continue
            entry = f'{class_name}-{meth}({args})'
            if entry not in calls:
                calls.append(entry)

        # Direct calls: FuncName(args) — no object/chain prefix; assume same class
        # Strip obj-call, chain, template spans first to avoid re-matching their method names
        scrubbed = RE_TEMPLATE_CALL.sub('', RE_CHAIN_CALL.sub('', RE_OBJ_CALL.sub('', line)))
        for dm in RE_DIRECT_CALL.finditer(scrubbed):
            meth = dm.group(1)
            args = dm.group(2).strip()
            if meth in SKIP_KEYWORDS or _is_macro(meth) or meth in IGNORE_FUNCS:
                continue
            entry = f'{class_name}-{meth}({args})'
            if entry not in calls:
                calls.append(entry)

    return calls


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Trace test function calls from command list.')
    parser.add_argument('--cmd', default='testtool/command.txt',
                        help='Path to command.txt')
    parser.add_argument('--src', default='testtool/DBMDownLoadTest.cpp',
                        help='Path to DBMDownLoadTest.cpp')
    parser.add_argument('--out', default='',
                        help='Write output to file instead of stdout')
    args = parser.parse_args()

    commands  = read_commands(args.cmd)
    gfuncs    = build_gfuncs_map(args.src)

    if not commands:
        print('[run_command] No commands to process.')
        return

    output_lines = []

    for cmd in commands:
        cpp_func = gfuncs.get(cmd)
        if not cpp_func:
            output_lines.append(f'[{cmd}] -> NOT FOUND in g_funcs')
            continue

        result = extract_body(cpp_func, args.src)
        if result is None:
            output_lines.append(f'[{cmd} -> {cpp_func}] -> body not found')
            continue

        params_str, body_lines, class_name = result
        calls = extract_calls(params_str, body_lines, class_name)

        output_lines.append(f'[{cmd} -> {cpp_func}]')
        if calls:
            for c in calls:
                output_lines.append(f'  {c}')
        else:
            output_lines.append('  (no resolvable calls)')

    output = '\n'.join(output_lines)

    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            fh.write(output + '\n')
        print(f'[run_command] written to {args.out}')
    else:
        print(output)


if __name__ == '__main__':
    main()
