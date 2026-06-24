# coding: utf-8
"""
main.py  —  Call-tree tracer

Pipeline:
  1. Read command.txt  (one command name per line)
  2. run_command  : command -> C++ test func -> list of ClassName-Method(args) calls
  3. functions.txt: match each call by class+name+param_count -> find canonical signature
  4. analyze_function: recurse into matched function body -> more calls
  5. Stop when a called function is not in functions.txt  (leaf node)
  6. Write one  output/command_<Name>.txt  per command, tree format

Usage:
    python tools/main.py \
        --cmd   testtool/command.txt \
        --src   testtool/DBMDownLoadTest.cpp \
        --funcs functions.txt \
        --dbm   com/project/DBM \
        --out   output
"""

import os
import re
import sys
import argparse

# Make sure sibling tool files are importable
sys.path.insert(0, os.path.dirname(__file__))

from run_command     import read_commands, build_gfuncs_map, extract_body, extract_calls
from analyze_function import analyze_function


# ---------------------------------------------------------------------------
# Node — one function call in the tree
# ---------------------------------------------------------------------------

class Node:
    def __init__(self, label: str, resolved: bool = True):
        """
        label    : 'ClassName-FuncName(args)'
        resolved : True  -> found in functions.txt, children may exist
                   False -> leaf (not in functions.txt or already visited)
        """
        self.label    = label
        self.resolved = resolved
        self.children: list['Node'] = []

    def render(self, indent: int = 0, prefix: str = '') -> list:
        """Render the subtree as text lines (box-drawing style)."""
        tag = '' if self.resolved else '  [external]'
        lines = [f'{prefix}{self.label}{tag}']
        for i, child in enumerate(self.children):
            is_last   = (i == len(self.children) - 1)
            connector = '`-- ' if is_last else '|-- '
            extender  = '    ' if is_last else '|   '
            lines.extend(child.render(indent + 1, prefix + connector))
            # Replace the connector in subsequent lines of that child subtree
            # (already handled by recursive prefix extension)
        return lines


# ---------------------------------------------------------------------------
# functions.txt index
# ---------------------------------------------------------------------------

class FunctionIndex:
    """
    Loads functions.txt produced by extract_functions.py.
    Each line:  ClassName-FuncName(Type1, Type2)

    Lookup key: (class_name, func_name, param_count)
    Returns the canonical signature string on match.
    """

    def __init__(self, path: str):
        self._index: dict[tuple, str] = {}   # (cls, func, nparams) -> full_line
        self._load(path)

    def _load(self, path: str):
        try:
            with open(path, encoding='utf-8', errors='replace') as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    m = re.match(r'^(\w*)-(\w+)\(([^)]*)\)$', line)
                    if not m:
                        continue
                    cls   = m.group(1)
                    func  = m.group(2)
                    raw_p = m.group(3).strip()
                    n_params = 0 if not raw_p else len(raw_p.split(','))
                    key = (cls, func, n_params)
                    if key not in self._index:
                        self._index[key] = line
        except FileNotFoundError:
            print(f'[main] WARNING: functions.txt not found: {path}')

    def lookup(self, class_name: str, func_name: str, args_str: str) -> str | None:
        """
        args_str is the raw argument string from the call site, e.g. '1' or 'db, cond'.
        We count commas+1 (or 0 for empty) to determine param count.
        Returns canonical signature or None.
        """
        args_str = args_str.strip()
        n = 0 if not args_str else len(args_str.split(','))
        result = self._index.get((class_name, func_name, n))
        if result is None and class_name:
            result = self._index.get(('', func_name, n))
        return result

    def lookup_fuzzy(self, class_name: str, func_name: str) -> str | None:
        """Fallback: match by class+name only, ignoring param count.
        Used when analyze_function loses arg-count info on direct/recursive calls."""
        for (cls, func, _), sig in self._index.items():
            if cls == class_name and func == func_name:
                return sig
        if class_name:
            for (cls, func, _), sig in self._index.items():
                if cls == '' and func == func_name:
                    return sig
        return None

    def parse_canonical(self, sig: str) -> tuple:
        """'ClassName-FuncName(Type1, Type2)' -> (class, func, [types])"""
        m = re.match(r'^(\w*)-(\w+)\(([^)]*)\)$', sig)
        if not m:
            return ('', '', [])
        types = [t.strip() for t in m.group(3).split(',') if t.strip()]
        return m.group(1), m.group(2), types


# ---------------------------------------------------------------------------
# Call parser  (parse a call-site label into components)
# ---------------------------------------------------------------------------

RE_CALL_LABEL = re.compile(r'^(\w*)-(\w+)\(([^)]*)\)$')  # class may be empty

def parse_call_label(label: str):
    """
    'DBMstdFoo-Bar(x, y)'  ->  ('DBMstdFoo', 'Bar', 'x, y')
    '-FreeFunc(z)'         ->  ('', 'FreeFunc', 'z')
    Returns None on parse failure.
    """
    m = RE_CALL_LABEL.match(label.strip())
    if not m:
        return None
    return m.group(1), m.group(2), m.group(3)


# ---------------------------------------------------------------------------
# Recursive tree builder
# ---------------------------------------------------------------------------

def build_tree(call_label: str,
               func_index: FunctionIndex,
               dbm_root: str,
               visited: set,
               depth: int = 0,
               max_depth: int = 20) -> Node:
    """
    Recursively build a Node tree for call_label.
    visited : set of labels already expanded (cycle guard)
    """
    if depth > max_depth:
        return Node(call_label + '  [max depth]', resolved=False)

    parsed = parse_call_label(call_label)
    if not parsed:
        return Node(call_label, resolved=False)

    class_name, func_name, args_str = parsed

    # Look up in functions.txt — canonical has proper param types
    canonical = func_index.lookup(class_name, func_name, args_str)
    if canonical is None and class_name:
        # Arg count mismatch (e.g. direct/recursive call loses param info):
        # try fuzzy match by class+name so recursive calls show [recursive]
        # instead of [external], and sibling calls are still explored.
        canonical = func_index.lookup_fuzzy(class_name, func_name)
    if canonical is None:
        # Not in known functions — show call-site label as leaf
        return Node(call_label, resolved=False)

    # Use canonical label (ClassName-FuncName(Type1, Type2)) instead of call-site args
    node = Node(canonical, resolved=True)

    # Cycle guard
    if canonical in visited:
        node.label += '  [recursive]'
        return node
    visited = visited | {canonical}

    # Get param types for analyze_function
    _, _, param_types = func_index.parse_canonical(canonical)

    # Extract calls inside this function body
    inner_calls = analyze_function(
        class_name  = class_name,
        func_name   = func_name,
        param_types = param_types,
        src_root    = dbm_root,
    )

    for call in inner_calls:
        child = build_tree(call, func_index, dbm_root, visited, depth + 1, max_depth)
        node.children.append(child)

    return node


# ---------------------------------------------------------------------------
# Tree renderer (full tree -> text lines)
# ---------------------------------------------------------------------------

def render_tree(root: Node) -> list:
    """Render tree with box-drawing characters."""
    def _render(node: Node, prefix: str, is_last: bool) -> list:
        connector = '`-- ' if is_last else '|-- '
        tag = '' if node.resolved else '  [external]'
        header = f'{prefix}{connector}{node.label}{tag}'
        lines = [header]
        child_prefix = prefix + ('    ' if is_last else '|   ')
        for i, child in enumerate(node.children):
            lines.extend(_render(child, child_prefix, i == len(node.children) - 1))
        return lines

    # Root node (no connector)
    tag = '' if root.resolved else '  [external]'
    result = [f'{root.label}{tag}']
    for i, child in enumerate(root.children):
        result.extend(_render(child, '', i == len(root.children) - 1))
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Recursive call-tree tracer.')
    parser.add_argument('--cmd',   default='testtool/command.txt',
                        help='command.txt path')
    parser.add_argument('--src',   default='testtool/DBMDownLoadTest.cpp',
                        help='DBMDownLoadTest.cpp path')
    parser.add_argument('--funcs', default='functions.txt',
                        help='functions.txt produced by extract_functions.py')
    parser.add_argument('--dbm',   default='com/project/DBM',
                        help='Root folder of DBM C++ source (for analyze_function)')
    parser.add_argument('--out',   default='output',
                        help='Output folder for command_xxx.txt files')
    parser.add_argument('--depth', type=int, default=20,
                        help='Max recursion depth (default 20)')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    commands  = read_commands(args.cmd)
    gfuncs    = build_gfuncs_map(args.src)
    func_idx  = FunctionIndex(args.funcs)

    for cmd in commands:
        cpp_func = gfuncs.get(cmd)
        if not cpp_func:
            print(f'[{cmd}] not found in g_funcs — skipping')
            continue

        result = extract_body(cpp_func, args.src)
        if result is None:
            print(f'[{cmd}] body of {cpp_func} not found — skipping')
            continue

        params_str, body_lines, class_name = result
        top_calls = extract_calls(params_str, body_lines, class_name, src_root=args.dbm)

        # Root node represents the command / test function itself
        root = Node(f'{cmd} [{cpp_func}]', resolved=True)
        visited_global: set = set()

        for call_label in top_calls:
            child = build_tree(call_label, func_idx, args.dbm,
                               visited_global, depth=0, max_depth=args.depth)
            root.children.append(child)

        tree_lines = render_tree(root)
        out_path = os.path.join(args.out, f'command_{cmd}.txt')
        with open(out_path, 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(tree_lines) + '\n')
        print(f'[{cmd}] -> {out_path}  ({len(tree_lines)} lines)')

    print('Done.')


if __name__ == '__main__':
    main()
